
#define DT_DRV_COMPAT zmk_rolling_combos

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>
#include <zmk/matrix.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>
#include <zmk/events/sensor_event.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * Asynchronous activity poke.  This module's event subscription is linked
 * before the activity listener (external module vs app code), so captured
 * position events never reach activity — leaving the idle timer stale.
 *
 * We submit a deferred work item that raises a zmk_sensor_event.  The
 * activity listener subscribes to sensor events and calls note_activity(),
 * resetting the idle timestamp.  No sensor behaviors are configured on
 * keyboards without encoders, so the event is otherwise harmless.  Using
 * a work item avoids re-entrancy with the event dispatch loop.
 */
static void activity_poke_work_handler(struct k_work *work) {
    raise_zmk_sensor_event((struct zmk_sensor_event){
        .sensor_index = UINT8_MAX,
        .channel_data_size = 0,
        .timestamp = k_uptime_get()});
}
static K_WORK_DEFINE(activity_poke_work, activity_poke_work_handler);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/*
 * ─── Compile-time combo extraction from devicetree ───────────────────────────
 */

#define COMBOS_KEYS_BYTE_ARRAY(node_id)                                                            \
    uint8_t _CONCAT(combo_prop_, node_id)[DT_PROP_LEN(node_id, key_positions)];

#define MAX_COMBO_KEYS sizeof(union { DT_INST_FOREACH_CHILD(0, COMBOS_KEYS_BYTE_ARRAY) })

struct combo_cfg {
    int32_t key_positions[MAX_COMBO_KEYS];
    int16_t key_position_len;
    int16_t require_prior_idle_ms;
    int32_t timeout_ms;
    uint32_t layer_mask;
    struct zmk_behavior_binding behavior;
    bool slow_release;
};

struct active_combo {
    uint16_t combo_idx;
    uint16_t key_positions_pressed_count;
    struct zmk_position_state_changed_event key_positions_pressed[MAX_COMBO_KEYS];
};

/*
 * ─── DT macros for populating the static combo array ─────────────────────────
 */

#define PROP_BIT_AT_IDX(n, prop, idx) BIT(DT_PROP_BY_IDX(n, prop, idx))

#define NODE_PROP_BITMASK(n, prop)                                                                 \
    COND_CODE_1(DT_NODE_HAS_PROP(n, prop),                                                         \
                (DT_FOREACH_PROP_ELEM_SEP(n, prop, PROP_BIT_AT_IDX, (|))), (0))

#define COMBO_INST(n, positions)                                                                   \
    COND_CODE_1(IS_EQ(DT_PROP_LEN(n, key_positions), positions),                                   \
                ({                                                                                 \
                    .timeout_ms = DT_PROP(n, timeout_ms),                                          \
                    .require_prior_idle_ms = DT_PROP(n, require_prior_idle_ms),                    \
                    .key_positions = DT_PROP(n, key_positions),                                    \
                    .key_position_len = DT_PROP_LEN(n, key_positions),                             \
                    .behavior = ZMK_KEYMAP_EXTRACT_BINDING(0, n),                                  \
                    .slow_release = DT_PROP(n, slow_release),                                      \
                    .layer_mask = NODE_PROP_BITMASK(n, layers),                                    \
                },),                                                                               \
                ())

#define COMBO_CONFIGS_WITH_MATCHING_POSITIONS_LEN(positions, _ignore)                              \
    DT_INST_FOREACH_CHILD_VARGS(0, COMBO_INST, positions)

/*
 * Combos are sorted shortest-first by key_position_len.
 * 20 is chosen as a reasonable upper bound (10 fingers × 2 keys per finger).
 */
static const struct combo_cfg combos[] = {
    LISTIFY(20, COMBO_CONFIGS_WITH_MATCHING_POSITIONS_LEN, (), 0)};

_Static_assert(ARRAY_SIZE(combos) < UINT16_MAX, "combo count must be less than UINT16_MAX sentinel");

#define COMBO_ONE(n) +1
#define COMBO_CHILDREN_COUNT (0 DT_INST_FOREACH_CHILD(0, COMBO_ONE))

/* Bitmask sizing: one bit per combo, rounded up to 32-bit words. */
#define BYTES_FOR_COMBOS_MASK DIV_ROUND_UP(COMBO_CHILDREN_COUNT, 32)

/*
 * Virtual key positions for rolling-combos.  Offset by 1000 past ZMK_KEYMAP_LEN
 * to avoid collisions with ZMK's internal virtual positions (sensors, stock
 * combos, input processors).
 */
#define ROLLING_COMBOS_VIRTUAL_KEY_POSITION(index) (ZMK_KEYMAP_LEN + 1000 + (index))

/*
 * ─── Detection context: one per concurrent combo detection ───────────────────
 *
 * The stock combo engine uses a single set of globals for detection state.
 * We replace that with an array of independent contexts so that multiple
 * combo detections can proceed simultaneously (combo rolling).
 */
struct combo_detection_ctx {
    bool active;
    uint8_t pressed_keys_count;
    struct zmk_position_state_changed_event pressed_keys[MAX_COMBO_KEYS];
    uint32_t candidates[BYTES_FOR_COMBOS_MASK];
    int16_t fully_pressed_combo;
    struct k_work_delayable timeout_task;
    int64_t timeout_task_timeout_at;
};

static void check_and_activate_complete_combos(struct combo_detection_ctx *ctx);

/*
 * ─── Global state ────────────────────────────────────────────────────────────
 */

/* Per-key-position → combo bitmask lookup table */
static uint32_t combo_lookup[ZMK_KEYMAP_LEN][BYTES_FOR_COMBOS_MASK] = {};

/* Detection contexts (concurrent combo detection slots) */
static struct combo_detection_ctx
    detection_contexts[CONFIG_ZMK_ROLLING_COMBOS_MAX_DETECTION_CONTEXTS];

/* Active (already triggered) combos */
static struct active_combo active_combos[CONFIG_ZMK_ROLLING_COMBOS_MAX_PRESSED_COMBOS] = {};
static uint8_t active_combo_count = 0;

/* Timestamp tracking for require-prior-idle-ms */
static int64_t last_tapped_timestamp = INT32_MIN;
static int64_t last_combo_timestamp = INT32_MIN;

/*
 * ─── Helper functions (ported from stock combo.c) ────────────────────────────
 */

static void store_last_tapped(int64_t timestamp) {
    if (timestamp > last_combo_timestamp) {
        last_tapped_timestamp = timestamp;
    }
}

static int initialize_combo(size_t index) {
    const struct combo_cfg *new_combo = &combos[index];
    for (size_t kp = 0; kp < new_combo->key_position_len; kp++) {
        sys_bitfield_set_bit((mem_addr_t)&combo_lookup[new_combo->key_positions[kp]], index);
    }
    return 0;
}

static bool combo_active_on_layer(const struct combo_cfg *combo, uint8_t layer) {
    if (!combo->layer_mask) {
        return true;
    }
    return combo->layer_mask & BIT(layer);
}

static bool is_quick_tap(const struct combo_cfg *combo, int64_t timestamp) {
    return (last_tapped_timestamp + combo->require_prior_idle_ms) > timestamp;
}

static inline uint8_t zero_one_or_more_bits(uint32_t field) {
    if (field == 0) {
        return 0;
    }
    if ((field & (field - 1)) == 0) {
        return 1;
    }
    return 2;
}

/*
 * ─── Active combo management ─────────────────────────────────────────────────
 */

static struct active_combo *store_active_combo(int32_t combo_idx) {
    for (int i = 0; i < CONFIG_ZMK_ROLLING_COMBOS_MAX_PRESSED_COMBOS; i++) {
        if (active_combos[i].combo_idx == UINT16_MAX) {
            active_combos[i].combo_idx = combo_idx;
            active_combo_count++;
            return &active_combos[i];
        }
    }
    LOG_ERR("Unable to store combo; already %d active. Increase "
            "CONFIG_ZMK_ROLLING_COMBOS_MAX_PRESSED_COMBOS",
            CONFIG_ZMK_ROLLING_COMBOS_MAX_PRESSED_COMBOS);
    return NULL;
}

static void deactivate_combo(int active_combo_index) {
    if (active_combo_count == 0) {
        return;
    }
    active_combo_count--;
    if (active_combo_index != active_combo_count) {
        memcpy(&active_combos[active_combo_index], &active_combos[active_combo_count],
               sizeof(struct active_combo));
    }
    active_combos[active_combo_count] = (struct active_combo){0};
    active_combos[active_combo_count].combo_idx = UINT16_MAX;
}

static inline int press_combo_behavior(int combo_idx, const struct combo_cfg *combo,
                                       int32_t timestamp) {
    struct zmk_behavior_binding_event event = {
        .position = ROLLING_COMBOS_VIRTUAL_KEY_POSITION(combo_idx),
        .timestamp = timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    last_combo_timestamp = timestamp;

    return zmk_behavior_invoke_binding(&combo->behavior, event, true);
}

static inline int release_combo_behavior(int combo_idx, const struct combo_cfg *combo,
                                         int32_t timestamp) {
    struct zmk_behavior_binding_event event = {
        .position = ROLLING_COMBOS_VIRTUAL_KEY_POSITION(combo_idx),
        .timestamp = timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    return zmk_behavior_invoke_binding(&combo->behavior, event, false);
}

/**
 * Check whether a key position is currently held in any active (already
 * triggered) combo.  This enables the overlap/rolling feature: a new combo
 * detection context can count keys held by prior combos towards completeness.
 */
static bool is_position_held_in_active_combo(int32_t position) {
    for (int i = 0; i < active_combo_count; i++) {
        struct active_combo *ac = &active_combos[i];
        for (int k = 0; k < ac->key_positions_pressed_count; k++) {
            if (ac->key_positions_pressed[k].data.position == position) {
                return true;
            }
        }
    }
    return false;
}

/**
 * Release a key from ALL matching active combos.  Unlike stock combo.c which
 * returns after the first match, we continue through all active combos because
 * rolling combos can share key positions.
 */
static bool release_combo_key(int32_t position, int64_t timestamp) {
    bool key_released_any = false;

    for (int combo_idx = 0; combo_idx < active_combo_count; /* see below */) {
        struct active_combo *active_combo = &active_combos[combo_idx];

        bool key_released = false;
        bool all_keys_pressed = active_combo->key_positions_pressed_count ==
                                combos[active_combo->combo_idx].key_position_len;
        bool all_keys_released = true;

        for (int i = 0; i < active_combo->key_positions_pressed_count; i++) {
            if (key_released) {
                active_combo->key_positions_pressed[i - 1] =
                    active_combo->key_positions_pressed[i];
                all_keys_released = false;
            } else if (active_combo->key_positions_pressed[i].data.position != position) {
                all_keys_released = false;
            } else {
                key_released = true;
            }
        }

        if (key_released) {
            active_combo->key_positions_pressed_count--;
            key_released_any = true;

            const struct combo_cfg *c = &combos[active_combo->combo_idx];
            if ((c->slow_release && all_keys_released) ||
                (!c->slow_release && all_keys_pressed)) {
                release_combo_behavior(active_combo->combo_idx, c, timestamp);
            }

            if (all_keys_released) {
                deactivate_combo(combo_idx);
                /* Array shifted; don't increment index */
                continue;
            }
        }

        combo_idx++;
    }

    return key_released_any;
}

/*
 * ─── Detection context management ────────────────────────────────────────────
 */

static struct combo_detection_ctx *find_free_context(void) {
    for (int i = 0; i < CONFIG_ZMK_ROLLING_COMBOS_MAX_DETECTION_CONTEXTS; i++) {
        if (!detection_contexts[i].active) {
            return &detection_contexts[i];
        }
    }
    return NULL;
}

static void init_context(struct combo_detection_ctx *ctx) {
    k_work_cancel_delayable(&ctx->timeout_task);
    ctx->active = true;
    ctx->pressed_keys_count = 0;
    memset(ctx->candidates, 0, sizeof(ctx->candidates));
    ctx->fully_pressed_combo = INT16_MAX;
    ctx->timeout_task_timeout_at = 0;
}

static void destroy_context(struct combo_detection_ctx *ctx) {
    k_work_cancel_delayable(&ctx->timeout_task);
    ctx->active = false;
    ctx->pressed_keys_count = 0;
    memset(ctx->candidates, 0, sizeof(ctx->candidates));
    ctx->fully_pressed_combo = INT16_MAX;
    ctx->timeout_task_timeout_at = 0;
}

/*
 * ─── Context-local detection functions ───────────────────────────────────────
 */

/**
 * Set up candidates for the first keypress in a context.
 * @param excluded  Bitmask of combo indices already tracked by other contexts.
 */
static int ctx_setup_candidates_for_first_keypress(struct combo_detection_ctx *ctx,
                                                   int32_t position, int64_t timestamp,
                                                   const uint32_t *excluded) {
    int count = 0;
    uint8_t highest_active_layer = zmk_keymap_highest_layer_active();

    for (size_t i = 0; i < ARRAY_SIZE(combos); i++) {
        if (sys_bitfield_test_bit((mem_addr_t)&combo_lookup[position], i)) {
            /* Skip combos being tracked by another context */
            if (sys_bitfield_test_bit((mem_addr_t)excluded, i)) {
                continue;
            }

            const struct combo_cfg *combo = &combos[i];
            if (combo_active_on_layer(combo, highest_active_layer) &&
                !is_quick_tap(combo, timestamp)) {
                sys_bitfield_set_bit((mem_addr_t)&ctx->candidates, i);
                count++;
            }
        }
    }

    return count;
}

static int ctx_filter_candidates(struct combo_detection_ctx *ctx, int32_t position) {
    int matches = 0;
    for (int i = 0; i < BYTES_FOR_COMBOS_MASK; i++) {
        ctx->candidates[i] &= combo_lookup[position][i];
        if (matches < 2) {
            matches += zero_one_or_more_bits(ctx->candidates[i]);
        }
    }
    return matches;
}

static int ctx_filter_timed_out_candidates(struct combo_detection_ctx *ctx, int64_t timestamp) {
    if (ctx->pressed_keys_count == 0) {
        return 0;
    }

    int remaining = 0;
    for (size_t i = 0; i < ARRAY_SIZE(combos); i++) {
        if (sys_bitfield_test_bit((mem_addr_t)&ctx->candidates, i)) {
            if (ctx->pressed_keys[0].data.timestamp + combos[i].timeout_ms > timestamp) {
                remaining++;
            } else {
                sys_bitfield_clear_bit((mem_addr_t)&ctx->candidates, i);
            }
        }
    }

    return remaining;
}

static int ctx_capture_pressed_key(struct combo_detection_ctx *ctx,
                                   const struct zmk_position_state_changed *ev) {
    if (ctx->pressed_keys_count == MAX_COMBO_KEYS) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    ctx->pressed_keys[ctx->pressed_keys_count++] = copy_raised_zmk_position_state_changed(ev);
    return ZMK_EV_EVENT_CAPTURED;
}

static int64_t ctx_first_candidate_timeout(struct combo_detection_ctx *ctx) {
    if (ctx->pressed_keys_count == 0) {
        return LONG_MAX;
    }

    int64_t first_timeout = LONG_MAX;
    for (size_t i = 0; i < ARRAY_SIZE(combos); i++) {
        if (sys_bitfield_test_bit((mem_addr_t)&ctx->candidates, i)) {
            first_timeout = MIN(first_timeout, combos[i].timeout_ms);
        }
    }

    return ctx->pressed_keys[0].data.timestamp + first_timeout;
}

static void ctx_update_timeout_task(struct combo_detection_ctx *ctx) {
    int64_t first_timeout = ctx_first_candidate_timeout(ctx);
    if (ctx->timeout_task_timeout_at == first_timeout) {
        return;
    }
    if (first_timeout == LONG_MAX) {
        ctx->timeout_task_timeout_at = 0;
        k_work_cancel_delayable(&ctx->timeout_task);
        return;
    }
    if (k_work_schedule(&ctx->timeout_task, K_MSEC(first_timeout - k_uptime_get())) >= 0) {
        ctx->timeout_task_timeout_at = first_timeout;
    }
}

/**
 * Enhanced completeness check.  A combo is considered "completely pressed" when
 * every key in its key_positions list is either:
 *   (a) captured in this detection context's pressed_keys buffer, OR
 *   (b) currently held in an already-activated combo (the rolling/overlap case).
 */
static bool candidate_is_completely_pressed_with_active(const struct combo_cfg *candidate,
                                                        struct combo_detection_ctx *ctx) {
    for (int k = 0; k < candidate->key_position_len; k++) {
        int32_t pos = candidate->key_positions[k];
        bool found = false;

        /* Check in this context's pressed_keys */
        for (int p = 0; p < ctx->pressed_keys_count; p++) {
            if (ctx->pressed_keys[p].data.position == pos) {
                found = true;
                break;
            }
        }

        /* Check in active combos (rolling overlap) */
        if (!found) {
            found = is_position_held_in_active_combo(pos);
        }

        if (!found) {
            return false;
        }
    }
    return true;
}

const struct zmk_listener zmk_listener_rolling_combos;

/**
 * Release all pressed keys from a context back to the event pipeline.
 * Event 0 is released (continues past our listener); events 1..N are
 * re-raised (re-enter from listener index 0) so they can potentially
 * start new combo detection contexts.
 */
static int ctx_release_pressed_keys(struct combo_detection_ctx *ctx) {
    uint8_t count = ctx->pressed_keys_count;
    ctx->pressed_keys_count = 0;
    for (int i = 0; i < count; i++) {
        struct zmk_position_state_changed_event *ev = &ctx->pressed_keys[i];
        if (i == 0) {
            LOG_DBG("rolling_combos: releasing position event %d", ev->data.position);
            ZMK_EVENT_RELEASE(*ev);
        } else {
            LOG_DBG("rolling_combos: reraising position event %d", ev->data.position);
            ZMK_EVENT_RAISE(*ev);
        }
    }
    return count;
}

/**
 * Activate a combo from a detection context: store it as active, transfer
 * the pressed keys, and invoke the behavior's press handler.
 *
 * For rolling/overlap combos, some key positions may be "borrowed" from
 * already-active combos rather than captured in this context.  We register
 * ALL key positions in the new active combo so that release_combo_key can
 * properly handle them.
 */
static void activate_combo_from_ctx(int combo_idx, struct combo_detection_ctx *ctx) {
    struct active_combo *ac = store_active_combo(combo_idx);
    if (ac == NULL) {
        ctx_release_pressed_keys(ctx);
        return;
    }

    const struct combo_cfg *cfg = &combos[combo_idx];
    ac->key_positions_pressed_count = 0;

    /*
     * First, move keys from the context's pressed_keys that are part of
     * this combo into the active combo.
     */
    int remaining = 0;
    for (int i = 0; i < ctx->pressed_keys_count; i++) {
        bool part_of_combo = false;
        for (int k = 0; k < cfg->key_position_len; k++) {
            if (ctx->pressed_keys[i].data.position == cfg->key_positions[k]) {
                part_of_combo = true;
                break;
            }
        }

        if (part_of_combo && ac->key_positions_pressed_count < MAX_COMBO_KEYS) {
            ac->key_positions_pressed[ac->key_positions_pressed_count++] = ctx->pressed_keys[i];
        } else {
            /* Keep non-combo keys in the context for later release */
            ctx->pressed_keys[remaining++] = ctx->pressed_keys[i];
        }
    }
    ctx->pressed_keys_count = remaining;

    /*
     * Second, for borrowed positions (held in other active combos but not in
     * this context), create synthetic entries so release_combo_key can track
     * them.  We reuse the event data from the active combo that holds them.
     */
    for (int k = 0; k < cfg->key_position_len; k++) {
        int32_t pos = cfg->key_positions[k];

        /* Check if already added from context */
        bool already_added = false;
        for (int j = 0; j < ac->key_positions_pressed_count; j++) {
            if (ac->key_positions_pressed[j].data.position == pos) {
                already_added = true;
                break;
            }
        }
        if (already_added) {
            continue;
        }

        /* Find this position in another active combo and copy its event */
        for (int ci = 0; ci < active_combo_count; ci++) {
            struct active_combo *other = &active_combos[ci];
            if (other == ac) {
                continue;
            }
            for (int j = 0; j < other->key_positions_pressed_count; j++) {
                if (other->key_positions_pressed[j].data.position == pos) {
                    if (ac->key_positions_pressed_count < MAX_COMBO_KEYS) {
                        ac->key_positions_pressed[ac->key_positions_pressed_count++] =
                            other->key_positions_pressed[j];
                    }
                    goto next_key_position;
                }
            }
        }
    next_key_position:;
    }

    int64_t timestamp = ac->key_positions_pressed_count > 0
                            ? ac->key_positions_pressed[0].data.timestamp
                            : k_uptime_get();
    press_combo_behavior(combo_idx, cfg, (int32_t)timestamp);
}

/**
 * Clean up a detection context: cancel its timeout, activate a fully-pressed
 * combo if one was found, and release any remaining captured keys.
 */
static int ctx_cleanup(struct combo_detection_ctx *ctx) {
    k_work_cancel_delayable(&ctx->timeout_task);
    ctx->timeout_task_timeout_at = 0;
    memset(ctx->candidates, 0, sizeof(ctx->candidates));

    if (ctx->fully_pressed_combo != INT16_MAX) {
        activate_combo_from_ctx(ctx->fully_pressed_combo, ctx);
        ctx->fully_pressed_combo = INT16_MAX;
    }

    int released = ctx_release_pressed_keys(ctx);
    ctx->active = false;
    return released;
}

/*
 * ─── Timeout handler (shared by all contexts via CONTAINER_OF) ───────────────
 */

static void rolling_combos_timeout_handler(struct k_work *item) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(item);
    struct combo_detection_ctx *ctx =
        CONTAINER_OF(dwork, struct combo_detection_ctx, timeout_task);

    if (!ctx->active || ctx->timeout_task_timeout_at == 0 ||
        k_uptime_get() < ctx->timeout_task_timeout_at) {
        return;
    }

    if (ctx_filter_timed_out_candidates(ctx, ctx->timeout_task_timeout_at) == 0) {
        LOG_DBG("rolling_combos: context timeout, cleaning up");
        ctx_cleanup(ctx);
    } else if (ctx->active) {
        /*
         * After filtering timed-out candidates, re-check whether the
         * remaining candidates include one that is already fully pressed.
         * Without this, a combo that was complete but deferred (because
         * multiple candidates existed) would never activate after the
         * competing candidates time out.
         */
        check_and_activate_complete_combos(ctx);
        if (ctx->active) {
            ctx_update_timeout_task(ctx);
        }
    }
}

/*
 * ─── Core rolling-combos detection logic ─────────────────────────────────────────
 */

/**
 * Build an exclusion bitmask from all active detection contexts.
 * Combos already being tracked by another context are excluded so that
 * two contexts don't race to detect the same combo.
 */
static void build_exclusion_set(uint32_t *excluded,
                                struct combo_detection_ctx *skip_ctx) {
    memset(excluded, 0, BYTES_FOR_COMBOS_MASK * sizeof(uint32_t));
    for (int i = 0; i < CONFIG_ZMK_ROLLING_COMBOS_MAX_DETECTION_CONTEXTS; i++) {
        struct combo_detection_ctx *ctx = &detection_contexts[i];
        if (ctx->active && ctx != skip_ctx) {
            for (int j = 0; j < BYTES_FOR_COMBOS_MASK; j++) {
                excluded[j] |= ctx->candidates[j];
            }
        }
    }
}

/**
 * Find an active detection context whose candidate combos include the given
 * key position.  Returns NULL if no context wants this position.
 */
static struct combo_detection_ctx *find_context_for_position(int32_t position) {
    if (position < 0 || position >= ZMK_KEYMAP_LEN) {
        return NULL;
    }
    for (int i = 0; i < CONFIG_ZMK_ROLLING_COMBOS_MAX_DETECTION_CONTEXTS; i++) {
        struct combo_detection_ctx *ctx = &detection_contexts[i];
        if (!ctx->active) {
            continue;
        }

        /* Check if any of this context's candidates include this position */
        for (int j = 0; j < BYTES_FOR_COMBOS_MASK; j++) {
            if (ctx->candidates[j] & combo_lookup[position][j]) {
                return ctx;
            }
        }
    }
    return NULL;
}

/**
 * Check whether any candidate combo in this context is completely pressed
 * (accounting for keys held in active combos).  If the only remaining
 * candidate is complete, activate it immediately.
 */
static void check_and_activate_complete_combos(struct combo_detection_ctx *ctx) {
    int remaining_candidates = 0;
    int16_t complete_combo = INT16_MAX;

    for (size_t i = 0; i < ARRAY_SIZE(combos); i++) {
        if (!sys_bitfield_test_bit((mem_addr_t)&ctx->candidates, i)) {
            continue;
        }
        remaining_candidates++;

        if (candidate_is_completely_pressed_with_active(&combos[i], ctx)) {
            complete_combo = i;
        }
    }

    if (complete_combo != INT16_MAX) {
        ctx->fully_pressed_combo = complete_combo;
        if (remaining_candidates == 1) {
            /* Only one candidate and it's complete → activate now */
            ctx_cleanup(ctx);
        }
        /* If multiple candidates remain, defer until disambiguation */
    }
}

/**
 * Handle a key-down event.
 *
 * 1. Try to feed the key into an existing detection context.
 * 2. If no context wants it, try to start a new context.
 * 3. If neither works, bubble the event.
 */
static int position_state_down(const zmk_event_t *ev,
                               struct zmk_position_state_changed *data) {
    /* Bounds check: ignore positions outside our keymap */
    if (data->position >= ZMK_KEYMAP_LEN) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /*
     * Step 1: Check existing detection contexts for a match.
     */
    struct combo_detection_ctx *matching_ctx = find_context_for_position(data->position);

    if (matching_ctx) {
        ctx_filter_timed_out_candidates(matching_ctx, data->timestamp);
        int num_candidates = ctx_filter_candidates(matching_ctx, data->position);

        LOG_DBG("rolling_combos: feeding position %d into existing context", data->position);
        int ret = ctx_capture_pressed_key(matching_ctx, data);
        if (ret == ZMK_EV_EVENT_BUBBLE) {
            /* Buffer full — cannot capture.  Clean up the context and let
             * the event bubble so it is not silently dropped. */
            ctx_cleanup(matching_ctx);
            return ZMK_EV_EVENT_BUBBLE;
        }
        ctx_update_timeout_task(matching_ctx);

        if (num_candidates > 0) {
            check_and_activate_complete_combos(matching_ctx);
            return ret;
        } else {
            ctx_cleanup(matching_ctx);
            return ret;
        }
    }

    /*
     * Step 2: No existing context wants this key.  Try to start a new one.
     */
    struct combo_detection_ctx *new_ctx = find_free_context();
    if (!new_ctx) {
        LOG_DBG("rolling_combos: no free context for position %d", data->position);
        return ZMK_EV_EVENT_BUBBLE;
    }

    init_context(new_ctx);

    /* Build exclusion set: combos already tracked by other contexts */
    uint32_t excluded[BYTES_FOR_COMBOS_MASK];
    build_exclusion_set(excluded, new_ctx);

    int num_candidates = ctx_setup_candidates_for_first_keypress(new_ctx, data->position,
                                                                 data->timestamp, excluded);
    if (num_candidates == 0) {
        destroy_context(new_ctx);
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_DBG("rolling_combos: new context for position %d, %d candidates", data->position,
            num_candidates);
    int ret = ctx_capture_pressed_key(new_ctx, data);
    if (ret == ZMK_EV_EVENT_BUBBLE) {
        /* Should not happen for a fresh context, but handle defensively */
        destroy_context(new_ctx);
        return ZMK_EV_EVENT_BUBBLE;
    }
    ctx_update_timeout_task(new_ctx);

    check_and_activate_complete_combos(new_ctx);
    return ret;
}

/**
 * Handle a key-up event.
 *
 * 1. If the key is captured in any detection context, force cleanup of
 *    that context (resolving it to a combo or releasing keys).
 * 2. Release the key from all matching active combos.
 * 3. If neither, bubble the event.
 */
static int position_state_up(const zmk_event_t *ev,
                             struct zmk_position_state_changed *data) {
    /* Bounds check: ignore positions outside our keymap */
    if (data->position >= ZMK_KEYMAP_LEN) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /*
     * Step 1: Check detection contexts.
     */
    int released_keys = 0;
    for (int i = 0; i < CONFIG_ZMK_ROLLING_COMBOS_MAX_DETECTION_CONTEXTS; i++) {
        struct combo_detection_ctx *ctx = &detection_contexts[i];
        if (!ctx->active) {
            continue;
        }

        for (int k = 0; k < ctx->pressed_keys_count; k++) {
            if (ctx->pressed_keys[k].data.position == data->position) {
                LOG_DBG("rolling_combos: key-up position %d in context, cleaning up",
                        data->position);
                released_keys = ctx_cleanup(ctx);
                goto done_ctx_check;
            }
        }
    }
done_ctx_check:

    /*
     * Step 2: Check active combos.
     */
    bool combo_key_released = release_combo_key(data->position, data->timestamp);

    if (released_keys > 0 || combo_key_released) {
        if (released_keys > 0) {
            /*
             * ctx_cleanup released/re-raised captured key-DOWN events back to
             * the pipeline.  The key-UP that triggered this cleanup must also
             * be re-raised so that downstream listeners (keymap, hold-tap)
             * see a matching key-up for any re-released key-downs.
             */
            struct zmk_position_state_changed_event dupe_ev =
                copy_raised_zmk_position_state_changed(data);
            ZMK_EVENT_RAISE(dupe_ev);
            return ZMK_EV_EVENT_CAPTURED;
        }
        return ZMK_EV_EVENT_HANDLED;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

/*
 * ─── Event listener ──────────────────────────────────────────────────────────
 */

static int position_state_changed_listener(const zmk_event_t *ev) {
    struct zmk_position_state_changed *data = as_zmk_position_state_changed(ev);
    if (data == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (data->state) {
        k_work_submit(&activity_poke_work);
        return position_state_down(ev, data);
    } else {
        return position_state_up(ev, data);
    }
}

static int keycode_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (ev->state && !is_mod(ev->usage_page, ev->keycode)) {
        store_last_tapped(ev->timestamp);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

int behavior_rolling_combos_listener(const zmk_event_t *eh) {
    if (as_zmk_position_state_changed(eh) != NULL) {
        return position_state_changed_listener(eh);
    } else if (as_zmk_keycode_state_changed(eh) != NULL) {
        return keycode_state_changed_listener(eh);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(rolling_combos, behavior_rolling_combos_listener);
ZMK_SUBSCRIPTION(rolling_combos, zmk_position_state_changed);
ZMK_SUBSCRIPTION(rolling_combos, zmk_keycode_state_changed);

/*
 * ─── Initialization ──────────────────────────────────────────────────────────
 */

static int rolling_combos_init(void) {
    for (int i = 0; i < CONFIG_ZMK_ROLLING_COMBOS_MAX_PRESSED_COMBOS; i++) {
        active_combos[i].combo_idx = UINT16_MAX;
    }

    for (int i = 0; i < CONFIG_ZMK_ROLLING_COMBOS_MAX_DETECTION_CONTEXTS; i++) {
        detection_contexts[i].active = false;
        detection_contexts[i].fully_pressed_combo = INT16_MAX;
        k_work_init_delayable(&detection_contexts[i].timeout_task,
                              rolling_combos_timeout_handler);
    }

    LOG_DBG("rolling_combos: initializing %zu combos", ARRAY_SIZE(combos));
    for (size_t i = 0; i < ARRAY_SIZE(combos); i++) {
        initialize_combo(i);
    }

    return 0;
}

SYS_INIT(rolling_combos_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

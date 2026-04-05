# zmk-feature-rolling-combos

A ZMK module that extends the combo system with **combo rolling** — the ability to activate multiple combos whose key presses and releases overlap in time.

## The Problem

ZMK's built-in combo system uses a single global detection state. When you type fast and start pressing keys for a second combo before fully releasing the first combo's keys, detection fails. The second combo's keys either get absorbed into the first combo's state or pass through as individual keys.

## The Solution

This module provides a multi-context combo detection engine. Instead of one global detection buffer, it maintains multiple independent detection contexts that operate concurrently. Each context tracks its own candidate combos, pressed keys, and timeout — allowing several combo detections to proceed simultaneously.

The key innovation is **overlap awareness**: when checking if a combo is fully pressed, the engine also counts keys currently held in already-activated combos. This means a combo like `{1, 2}` can activate immediately if key 1 is held from a prior combo and key 2 is freshly pressed.

## Installation

Add the module to your `config/west.yml` manifest:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: your-username
      url-base: https://github.com/your-username
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-feature-rolling-combos
      remote: your-username
      revision: main
```

## Configuration

### Devicetree

Define your combos in a `rolling_combos` node instead of (or alongside) the stock `combos` node. The properties are identical to stock combos:

```dts
/ {
    rolling_combos {
        compatible = "zmk,rolling-combos";

        combo_esc {
            key-positions = <0 1>;
            bindings = <&kp ESC>;
            timeout-ms = <50>;
        };

        combo_tab {
            key-positions = <1 2>;
            bindings = <&kp TAB>;
            timeout-ms = <50>;
        };

        combo_enter {
            key-positions = <2 3>;
            bindings = <&kp ENTER>;
            timeout-ms = <40>;
            slow-release;
        };

        combo_layer {
            key-positions = <10 11>;
            bindings = <&mo 1>;
            layers = <0>;
            require-prior-idle-ms = <150>;
        };
    };
};
```

### Properties

| Property | Type | Required | Default | Description |
|---|---|---|---|---|
| `bindings` | phandle-array | Yes | — | Behavior to invoke when the combo triggers |
| `key-positions` | array | Yes | — | Key position indices that form the combo |
| `timeout-ms` | int | No | 50 | All keys must be pressed within this many ms |
| `require-prior-idle-ms` | int | No | -1 (disabled) | Minimum idle time since last keypress |
| `slow-release` | boolean | No | false | Release when all keys released (vs first key) |
| `layers` | array | No | all | Restrict combo to specific layers |

### Kconfig

Add to your `.conf` file if you need to adjust limits:

```ini
# Maximum simultaneously active (held) combos (default: 4)
CONFIG_ZMK_ROLLING_COMBOS_MAX_PRESSED_COMBOS=4

# Maximum concurrent combo detection slots (default: 3)
CONFIG_ZMK_ROLLING_COMBOS_MAX_DETECTION_CONTEXTS=3
```

## Migrating from Stock Combos

1. Remove or rename your existing `combos` node
2. Create a `rolling_combos` node with `compatible = "zmk,rolling-combos"`
3. Move your combo definitions into the new node (no property changes needed)

You can also keep both nodes if you want some combos with stock behavior and others with rolling support. The rolling-combos listener processes events first; keys that don't match any rolling-combos combo are passed through to the stock combo listener.

## How It Works

### Multi-Context Detection

When a key is pressed:

1. **Check existing contexts**: If any active detection context's candidates include this key position, feed the key into that context
2. **Start new context**: If no context wants the key, allocate a new detection context. Combos already tracked by other contexts are excluded to prevent racing
3. **Check completeness**: When checking if a combo is fully pressed, count keys in this context PLUS keys held in already-activated combos
4. **Bubble if no match**: If nothing matches, pass the event through

### Rolling Example

Combos: `ESC = {0, 1}`, `TAB = {1, 2}`

Sequence: press(0), press(1), press(2), release(0), release(1), release(2)

1. Press key 0 → new context A, candidates include ESC
2. Press key 1 → context A wants it, ESC fully pressed → activate ESC
3. Press key 2 → no active context wants it → new context B, candidates include TAB. Key 1 is held in active combo ESC → TAB is complete → activate TAB
4. Release key 0 → releases from active ESC combo → triggers ESC release (non-slow-release)
5. Release key 1 → releases from both ESC and TAB active combos
6. Release key 2 → releases from active TAB combo → deactivates TAB

Result: ESC press, TAB press, ESC release, TAB release — both combos fire correctly.

## License

MIT

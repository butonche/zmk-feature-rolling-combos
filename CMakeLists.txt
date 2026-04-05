
if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  target_sources_ifdef(CONFIG_ZMK_ROLLING_COMBOS app PRIVATE src/rolling_combos.c)
endif()

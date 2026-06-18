#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Per-domain Config umbrella — single source of truth for every tunable constant.
 *
 * The `namespace Config` body was carved into per-domain sub-headers under `config/` per
 * ADR-031 / Issue #39. Each sub-header reopens `namespace Config` and adds its own symbols;
 * modules MUST still source values from `Config::`, never hardcode. The `<Arduino.h>` /
 * `<stdint.h>` includes above are kept here so consumers continue to get them transitively
 * through `Config.h`.
 *
 * Include order is LOAD-BEARING: a sub-header that forwards a sibling's constant must follow
 * it (e.g. ConfigDrive's `TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE` needs ConfigGlobal;
 * ConfigLine's `CRUISE_SPEED = SPEED_NORMAL` needs ConfigDrive). ConfigChecks.h MUST be last —
 * its static_asserts reference symbols from every sub-header.
 */
#include "config/ConfigGlobal.h"
#include "config/ConfigDrive.h"
#include "config/ConfigLine.h"
#include "config/ConfigNav.h"
#include "config/ConfigMission.h"
#include "config/ConfigManipulator.h"
#include "config/ConfigVision.h"
#include "config/ConfigChecks.h"   // MUST be last

#endif    // CONFIG_H

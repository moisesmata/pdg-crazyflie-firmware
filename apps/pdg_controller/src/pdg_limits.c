/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "pdg_limits.h"
#include <math.h>

float pdgLimitCollective(float requested, bool enabled, float ceiling, bool *limited) {
  *limited = false;
  if (!isfinite(requested) || requested < 0.0f) return 0.0f;
  if (!enabled) return requested;
  if (!isfinite(ceiling) || ceiling < 0.0f) ceiling = 0.0f;
  if (ceiling > 65535.0f) ceiling = 65535.0f;
  *limited = requested > ceiling;
  return *limited ? ceiling : requested;
}

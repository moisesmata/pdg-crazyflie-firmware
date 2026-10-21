/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
/* Legacy command units, NOT Newtons or measured effectiveness. */
float pdgLimitCollective(float requested, bool enabled, float ceiling, bool *limited);

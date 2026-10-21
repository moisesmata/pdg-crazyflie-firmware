/* SPDX-License-Identifier: GPL-3.0-or-later
 * Thin out-of-tree tracker. Optimization and mission execution live on the host.
 * Pattern: Bitcraze app_out_of_tree_controller and A2R-Lab/tinympc-crazyflie.
 */
#include <stdint.h>
#include "autoconf.h"
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "controller.h"
#include "controller_mellinger.h"
#include "log.h"
#include "param.h"
#include "pdg_limits.h"
#define DEBUG_MODULE "PDG"
#include "debug.h"

static uint8_t capEnable = 0;  /* nominal operation by default */
static float maxCollective = 65535.0f;
static uint8_t capActive = 0;
static float requestedCollective = 0;
static float commandedCollective = 0;
static uint32_t updates = 0;
static uint8_t apiVersion = 1;
static uint8_t controllerId = ControllerTypeOot;

void appMain(void) {
  DEBUG_PRINT("Offboard PDG tracker ready; does not initiate flight.\n");
  while (1) vTaskDelay(M2T(1000));
}

void controllerOutOfTreeInit(void) {
  controllerMellingerFirmwareInit();
  updates = 0;
}

bool controllerOutOfTreeTest(void) { return controllerMellingerFirmwareTest(); }

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                         const sensorData_t *sensors, const state_t *state,
                         const stabilizerStep_t step) {
  /* Stock Mellinger handles its own update rate; do not apply the cap a second
     time on skipped ticks or change the fast-loop scheduling. */
  controllerMellingerFirmware(control, setpoint, sensors, state, step);
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, step)) return;
  updates++;
  capActive = 0;
  if (control->controlMode == controlModeLegacy) {
    bool limited = false;
    requestedCollective = control->thrust;
    control->thrust = pdgLimitCollective(control->thrust, capEnable != 0,
                                       maxCollective, &limited);
    commandedCollective = control->thrust;
    capActive = limited;
  }
}

PARAM_GROUP_START(pdg)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, apiVersion, &apiVersion)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, controllerId, &controllerId)
PARAM_ADD_CORE(PARAM_UINT8, capEnable, &capEnable)
PARAM_ADD_CORE(PARAM_FLOAT, maxCollective, &maxCollective)
PARAM_GROUP_STOP(pdg)

LOG_GROUP_START(pdg)
LOG_ADD_CORE(LOG_UINT32, updates, &updates)
LOG_ADD_CORE(LOG_UINT8, capActive, &capActive)
LOG_ADD_CORE(LOG_FLOAT, requested, &requestedCollective)
LOG_ADD_CORE(LOG_FLOAT, commanded, &commandedCollective)
LOG_GROUP_STOP(pdg)

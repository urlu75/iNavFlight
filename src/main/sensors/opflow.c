/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#ifdef OPTICAL_FLOW

#include "build_config.h"
#include "debug.h"

#include "common/maths.h"
#include "common/filter.h"

#include "scheduler/scheduler.h"

#include "config/config.h"
#include "config/runtime_config.h"

#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/opflow.h"

#include "sensors/sensors.h"
#include "sensors/opflow.h"
#include "sensors/battery.h"

// Driver access functions
opflow_t opflow;

// Optical flow output
opticalFlowData_t opticalFlow;

// Optical flow data LPF
static pt1Filter_t opflowVelXLpfState;
static pt1Filter_t opflowVelYLpfState;

#define OPTICAL_FLOW_LOWPASS_HZ         20.0f

void taskProcessOpticalFlow(void)
{
    const float opflowDt = getTaskDeltaTime(TASK_SELF) * 1e-6f;
    opflow_data_t driverFlowData;

    if (!sensors(SENSOR_OPTICAL_FLOW)) {
        return;
    }

    /* We receive raw motion data from the sensor (X - forward, Y - right coordinated), we need to compensate them for pitch and roll rate as aircraft rotation introduces change
     * in what the sensor sees and we receive that as "fake" motion. We also need to scale the motion according to FOV. We can't scale the motion to real units as we don't know the
     * altitude of the drone - we leave that high level processing to INAV Position Estimation core.
     */
    if (opflow.read((int16_t *)&driverFlowData)) {
        /*
        debug[0] = driverFlowData.delta.V.X;
        debug[1] = driverFlowData.delta.V.Y;
        debug[2] = driverFlowData.quality;
        */
    }
    else {
        driverFlowData.delta.V.X = 0;
        driverFlowData.delta.V.Y = 0;
    }

    opticalFlow.flowVelX = pt1FilterApply4(&opflowVelXLpfState, driverFlowData.delta.V.X, OPTICAL_FLOW_LOWPASS_HZ, opflowDt);
    opticalFlow.flowVelY = pt1FilterApply4(&opflowVelYLpfState, driverFlowData.delta.V.Y, OPTICAL_FLOW_LOWPASS_HZ, opflowDt);

    debug[0] = opticalFlow.flowVelX;
    debug[1] = opticalFlow.flowVelY;
    debug[2] = driverFlowData.quality;
}

#endif


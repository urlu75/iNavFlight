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

#include "config/config.h"
#include "config/runtime_config.h"

#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/opflow.h"

#include "sensors/sensors.h"
#include "sensors/opflow.h"
#include "sensors/battery.h"

opflow_t opflow;

void taskProcessOpticalFlow(void)
{
    opflow_data_t driverFlowData;

    if (!sensors(SENSOR_OPTICAL_FLOW)) {
        return;
    }

    if (opflow.read((int16_t *)&driverFlowData)) {
        debug[0] = driverFlowData.delta[0];
        debug[1] = driverFlowData.delta[1];
        debug[2] = driverFlowData.quality;
    }
}

#endif


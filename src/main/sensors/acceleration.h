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

#pragma once

// Type of accelerometer used/detected
typedef enum {
    ACC_DEFAULT = 0,
    ACC_NONE,
    ACC_ADXL345,
    ACC_MPU6050,
    ACC_MMA8452,
    ACC_BMA280,
    ACC_LSM303DLHC,
    ACC_MPU6000,
    ACC_MPU6500,
    ACC_MPU9250,
    ACC_FAKE,
    ACC_MAX = ACC_FAKE
} accelerationSensor_e;

extern sensor_align_e accAlign;
extern acc_t acc;

extern int32_t accADC[XYZ_AXIS_COUNT];

bool isAccelerationCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void updateAccelerationReadings(void);
void setAccelerationZero(flightDynamicsTrims_t * accZeroToUse);
void setAccelerationGain(flightDynamicsTrims_t * accGainToUse);
void setAccelerationFilter(int8_t initialAccLpfCutHz);

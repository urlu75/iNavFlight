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
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "scheduler/scheduler.h"
#include "scheduler/scheduler_tasks.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/rangefinder.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "io/asyncfatfs/asyncfatfs.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/hil.h"
#include "flight/failsafe.h"
#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

// June 2013     V2.2-dev

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)
#define GYRO_WATCHDOG_DELAY 100  // Watchdog for boards without interrupt for gyro

uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

float dT;

int16_t headFreeModeHold;

uint8_t motorControlEnable = false;

int16_t telemTemperature1;      // gyro sensor temperature
static uint32_t disarmAt;     // Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero

extern uint32_t currentTime;

static bool isRXDataNew;

bool isCalibrating(void)
{
#ifdef BARO
    if (sensors(SENSOR_BARO) && !isBaroCalibrationComplete()) {
        return true;
    }
#endif

    // Note: compass calibration is handled completely differently, outside of the main loop, see f.CALIBRATE_MAG

    return (!isAccelerationCalibrationComplete() && sensors(SENSOR_ACC)) || (!isGyroCalibrationComplete());
}

int16_t getAxisRcCommand(int16_t rawData, int16_t rate, int16_t deadband)
{
    int16_t stickDeflection;

    stickDeflection = constrain(rawData - masterConfig.rxConfig.midrc, -500, 500);
    stickDeflection = applyDeadband(stickDeflection, deadband);

    return rcLookup(stickDeflection, rate);
}

void annexCode(void)
{

    int32_t throttleValue;

    // Compute ROLL PITCH and YAW command
    rcCommand[ROLL] = getAxisRcCommand(rcData[ROLL], currentControlRateProfile->rcExpo8, currentProfile->rcControlsConfig.deadband);
    rcCommand[PITCH] = getAxisRcCommand(rcData[PITCH], currentControlRateProfile->rcExpo8, currentProfile->rcControlsConfig.deadband);
    rcCommand[YAW] = -getAxisRcCommand(rcData[YAW], currentControlRateProfile->rcYawExpo8, currentProfile->rcControlsConfig.yaw_deadband);

    //Compute THROTTLE command
    throttleValue = constrain(rcData[THROTTLE], masterConfig.rxConfig.mincheck, PWM_RANGE_MAX);
    throttleValue = (uint32_t)(throttleValue - masterConfig.rxConfig.mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - masterConfig.rxConfig.mincheck);       // [MINCHECK;2000] -> [0;1000]
    rcCommand[THROTTLE] = rcLookupThrottle(throttleValue);

    if (FLIGHT_MODE(HEADFREE_MODE)) {
        const float radDiff = degreesToRadians(DECIDEGREES_TO_DEGREES(attitude.values.yaw) - headFreeModeHold);
        const float cosDiff = cos_approx(radDiff);
        const float sinDiff = sin_approx(radDiff);
        const int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }

    if (ARMING_FLAG(ARMED)) {
        LED0_ON;
    } else {
        if (IS_RC_MODE_ACTIVE(BOXARM) == 0) {
            ENABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (!STATE(SMALL_ANGLE)) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }

        if (isCalibrating() || isSystemOverloaded()) {
            warningLedFlash();
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }

#if defined(NAV)
        if (naivationBlockArming()) {
            DISABLE_ARMING_FLAG(OK_TO_ARM);
        }
#endif

        if (ARMING_FLAG(OK_TO_ARM)) {
            warningLedDisable();
        } else {
            warningLedFlash();
        }

        warningLedUpdate();
    }

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);
}

void mwDisarm(void)
{
    if (ARMING_FLAG(ARMED)) {
        DISABLE_ARMING_FLAG(ARMED);

#ifdef BLACKBOX
        if (feature(FEATURE_BLACKBOX)) {
            finishBlackbox();
        }
#endif

        beeper(BEEPER_DISARMING);      // emit disarm tone
    }
}

#define TELEMETRY_FUNCTION_MASK (FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_SMARTPORT | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_MAVLINK)

void releaseSharedTelemetryPorts(void) {
    serialPort_t *sharedPort = findSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    while (sharedPort) {
        mspReleasePortIfAllocated(sharedPort);
        sharedPort = findNextSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    }
}

void mwArm(void)
{
    if (ARMING_FLAG(OK_TO_ARM)) {
        if (ARMING_FLAG(ARMED)) {
            return;
        }
        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            return;
        }
        if (!ARMING_FLAG(PREVENT_ARMING)) {
            ENABLE_ARMING_FLAG(ARMED);
            ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
            headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);

#ifdef BLACKBOX
            if (feature(FEATURE_BLACKBOX)) {
                serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
                if (sharedBlackboxAndMspPort) {
                    mspReleasePortIfAllocated(sharedBlackboxAndMspPort);
                }
                startBlackbox();
            }
#endif
            disarmAt = millis() + masterConfig.auto_disarm_delay * 1000;   // start disarm timeout, will be extended when throttle is nonzero

            //beep to indicate arming
#ifdef NAV
            if (navigationPositionEstimateIsHealthy())
                beeper(BEEPER_ARMING_GPS_FIX);
            else
                beeper(BEEPER_ARMING);
#else
            beeper(BEEPER_ARMING);
#endif

            return;
        }
    }

    if (!ARMING_FLAG(ARMED)) {
        beeperConfirmationBeeps(1);
    }
}

void processRx(void)
{
    static bool armedBeeperOn = false;

    calculateRxChannelsAndUpdateFailsafe(currentTime);

    // in 3D mode, we need to be able to disarm by switch at any time
    if (feature(FEATURE_3D)) {
        if (!IS_RC_MODE_ACTIVE(BOXARM))
            mwDisarm();
    }

    updateRSSI(currentTime);

    if (feature(FEATURE_FAILSAFE)) {

        if (currentTime > FAILSAFE_POWER_ON_DELAY_US && !failsafeIsMonitoring()) {
            failsafeStartMonitoring();
        }

        failsafeUpdateState();
    }

    throttleStatus_e throttleStatus = calculateThrottleStatus(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);

    // When armed and motors aren't spinning, do beeps and then disarm
    // board after delay so users without buzzer won't lose fingers.
    // mixTable constrains motor commands, so checking  throttleStatus is enough
    if (ARMING_FLAG(ARMED)
        && feature(FEATURE_MOTOR_STOP)
        && !STATE(FIXED_WING)
    ) {
        if (isUsingSticksForArming()) {
            if (throttleStatus == THROTTLE_LOW) {
                if (masterConfig.auto_disarm_delay != 0
                    && (int32_t)(disarmAt - millis()) < 0
                ) {
                    // auto-disarm configured and delay is over
                    mwDisarm();
                    armedBeeperOn = false;
                } else {
                    // still armed; do warning beeps while armed
                    beeper(BEEPER_ARMED);
                    armedBeeperOn = true;
                }
            } else {
                // throttle is not low
                if (masterConfig.auto_disarm_delay != 0) {
                    // extend disarm time
                    disarmAt = millis() + masterConfig.auto_disarm_delay * 1000;
                }

                if (armedBeeperOn) {
                    beeperSilence();
                    armedBeeperOn = false;
                }
            }
        } else {
            // arming is via AUX switch; beep while throttle low
            if (throttleStatus == THROTTLE_LOW) {
                beeper(BEEPER_ARMED);
                armedBeeperOn = true;
            } else if (armedBeeperOn) {
                beeperSilence();
                armedBeeperOn = false;
            }
        }
    }

    processRcStickPositions(&masterConfig.rxConfig, throttleStatus, masterConfig.disarm_kill_switch);

    updateActivatedModes(currentProfile->modeActivationConditions, currentProfile->modeActivationOperator);

    if (!cliMode) {
        updateAdjustmentStates(currentProfile->adjustmentRanges);
        processRcAdjustments(currentControlRateProfile, &masterConfig.rxConfig);
    }

    bool canUseHorizonMode = true;

    if ((IS_RC_MODE_ACTIVE(BOXANGLE) || (feature(FEATURE_FAILSAFE) && failsafeIsActive()) || naivationRequiresAngleMode()) && sensors(SENSOR_ACC)) {
        // bumpless transfer to Level mode
        canUseHorizonMode = false;

        if (!FLIGHT_MODE(ANGLE_MODE)) {
            ENABLE_FLIGHT_MODE(ANGLE_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(ANGLE_MODE); // failsafe support
    }

    if (IS_RC_MODE_ACTIVE(BOXHORIZON) && canUseHorizonMode) {

        DISABLE_FLIGHT_MODE(ANGLE_MODE);

        if (!FLIGHT_MODE(HORIZON_MODE)) {
            ENABLE_FLIGHT_MODE(HORIZON_MODE);
        }
    } else {
        DISABLE_FLIGHT_MODE(HORIZON_MODE);
    }

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        LED1_ON;
    } else {
        LED1_OFF;
    }

    /* Heading lock mode */
    if (IS_RC_MODE_ACTIVE(BOXHEADINGLOCK)) {
        if (!FLIGHT_MODE(HEADING_LOCK)) {
            ENABLE_FLIGHT_MODE(HEADING_LOCK);
        }
    } else {
        DISABLE_FLIGHT_MODE(HEADING_LOCK);
    }

    /* Flaperon mode */
    if (IS_RC_MODE_ACTIVE(BOXFLAPERON) && STATE(FLAPERON_AVAILABLE)) {
        if (!FLIGHT_MODE(FLAPERON)) {
            ENABLE_FLIGHT_MODE(FLAPERON);
        }
    } else {
        DISABLE_FLIGHT_MODE(FLAPERON);
    }

    /* Turn assistant mode */
    if (IS_RC_MODE_ACTIVE(BOXTURNASSIST)) {
        if (!FLIGHT_MODE(TURN_ASSISTANT)) {
            ENABLE_FLIGHT_MODE(TURN_ASSISTANT);
        }
    } else {
        DISABLE_FLIGHT_MODE(TURN_ASSISTANT);
    }

#if defined(MAG)
    if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
        if (IS_RC_MODE_ACTIVE(BOXMAG)) {
            if (!FLIGHT_MODE(MAG_MODE)) {
                ENABLE_FLIGHT_MODE(MAG_MODE);
                updateMagHoldHeading(DECIDEGREES_TO_DEGREES(attitude.values.yaw));
            }
        } else {
            DISABLE_FLIGHT_MODE(MAG_MODE);
        }
        if (IS_RC_MODE_ACTIVE(BOXHEADFREE)) {
            if (!FLIGHT_MODE(HEADFREE_MODE)) {
                ENABLE_FLIGHT_MODE(HEADFREE_MODE);
            }
        } else {
            DISABLE_FLIGHT_MODE(HEADFREE_MODE);
        }
        if (IS_RC_MODE_ACTIVE(BOXHEADADJ)) {
            headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw); // acquire new heading
        }
    }
#endif

    // Navigation may override PASSTHRU_MODE
    if (IS_RC_MODE_ACTIVE(BOXPASSTHRU) && !naivationRequiresAngleMode()) {
        ENABLE_FLIGHT_MODE(PASSTHRU_MODE);
    } else {
        DISABLE_FLIGHT_MODE(PASSTHRU_MODE);
    }

    /* In airmode Iterm should be prevented to grow when Low thottle and Roll + Pitch Centered.
       This is needed to prevent Iterm winding on the ground, but keep full stabilisation on 0 throttle while in air
       Low Throttle + roll and Pitch centered is assuming the copter is on the ground. Done to prevent complex air/ground detections */
    if (FLIGHT_MODE(PASSTHRU_MODE) || !ARMING_FLAG(ARMED)) {
        /* In PASSTHRU mode we reset integrators prevent I-term wind-up (PID output is not used in PASSTHRU) */
        pidResetErrorAccumulators();
    }
    else {
        if (throttleStatus == THROTTLE_LOW) {
            if (IS_RC_MODE_ACTIVE(BOXAIRMODE) && !failsafeIsActive() && ARMING_FLAG(ARMED)) {
                rollPitchStatus_e rollPitchStatus = calculateRollPitchCenterStatus(&masterConfig.rxConfig);

                // ANTI_WINDUP at centred stick with MOTOR_STOP is needed on MRs and not needed on FWs
                if ((rollPitchStatus == CENTERED) || (feature(FEATURE_MOTOR_STOP) && !STATE(FIXED_WING))) {
                    ENABLE_STATE(ANTI_WINDUP);
                }
                else {
                    DISABLE_STATE(ANTI_WINDUP);
                }
            }
            else {
                DISABLE_STATE(ANTI_WINDUP);
                pidResetErrorAccumulators();
            }
        }
        else {
            DISABLE_STATE(ANTI_WINDUP);
        }
    }

    if (masterConfig.mixerMode == MIXER_FLYING_WING || masterConfig.mixerMode == MIXER_AIRPLANE || masterConfig.mixerMode == MIXER_CUSTOM_AIRPLANE) {
        DISABLE_FLIGHT_MODE(HEADFREE_MODE);
    }

#ifdef TELEMETRY
    if (feature(FEATURE_TELEMETRY)) {
        if ((!masterConfig.telemetryConfig.telemetry_switch && ARMING_FLAG(ARMED)) ||
                (masterConfig.telemetryConfig.telemetry_switch && IS_RC_MODE_ACTIVE(BOXTELEMETRY))) {

            releaseSharedTelemetryPorts();
        } else {
            // the telemetry state must be checked immediately so that shared serial ports are released.
            telemetryCheckState();
            mspAllocateSerialPorts();
        }
    }
#endif

}

void filterRc(bool isRXDataNew)
{
    static int16_t lastCommand[4] = { 0, 0, 0, 0 };
    static int16_t deltaRC[4] = { 0, 0, 0, 0 };
    static int16_t factor, rcInterpolationFactor;
    static biquadFilter_t filteredCycleTimeState;
    static bool filterInitialised;
    uint16_t filteredCycleTime;
    uint16_t rxRefreshRate;

    // Set RC refresh rate for sampling and channels to filter
    initRxRefreshRate(&rxRefreshRate);

    // Calculate average cycle time (1Hz LPF on cycle time)
    if (!filterInitialised) {
        biquadFilterInit(&filteredCycleTimeState, 1, 0);
        filterInitialised = true;
    }

    filteredCycleTime = biquadFilterApply(&filteredCycleTimeState, (float) cycleTime);

    rcInterpolationFactor = rxRefreshRate / filteredCycleTime + 1;

    if (isRXDataNew) {
        for (int channel=0; channel < 4; channel++) {
            deltaRC[channel] = rcCommand[channel] -  (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);
            lastCommand[channel] = rcCommand[channel];
        }

        factor = rcInterpolationFactor - 1;
    } else {
        factor--;
    }

    // Interpolate steps of rcCommand
    if (factor > 0) {
        for (int channel=0; channel < 4; channel++) {
            rcCommand[channel] = lastCommand[channel] - deltaRC[channel] * factor/rcInterpolationFactor;
         }
    } else {
        factor = 0;
    }
}

void taskMainPidLoop(void)
{
    cycleTime = getTaskDeltaTime(TASK_SELF);
    dT = (float)cycleTime * 0.000001f;

    imuUpdateAccelerometer();
    imuUpdateGyroAndAttitude();

    annexCode();

    if (masterConfig.rxConfig.rcSmoothing) {
        filterRc(isRXDataNew);
    }

#if defined(NAV)
    if (isRXDataNew) {
        updateWaypointsAndNavigationMode();
    }
#endif

    isRXDataNew = false;

#if defined(NAV)
    updatePositionEstimator();
    applyWaypointNavigationAndAltitudeHold();
#endif

    // If we're armed, at minimum throttle, and we do arming via the
    // sticks, do not process yaw input from the rx.  We do this so the
    // motors do not spin up while we are trying to arm or disarm.
    // Allow yaw control for tricopters if the user wants the servo to move even when unarmed.
    if (isUsingSticksForArming() && rcData[THROTTLE] <= masterConfig.rxConfig.mincheck
#ifndef USE_QUAD_MIXER_ONLY
#ifdef USE_SERVOS
            && !((masterConfig.mixerMode == MIXER_TRI || masterConfig.mixerMode == MIXER_CUSTOM_TRI) && masterConfig.mixerConfig.tri_unarmed_servo)
#endif
            && masterConfig.mixerMode != MIXER_AIRPLANE
            && masterConfig.mixerMode != MIXER_FLYING_WING
            && masterConfig.mixerMode != MIXER_CUSTOM_AIRPLANE
#endif
    ) {
        rcCommand[YAW] = 0;
    }

    // Apply throttle tilt compensation
    if (!STATE(FIXED_WING)) {
        int16_t thrTiltCompStrength = 0;

        if (navigationRequiresThrottleTiltCompensation()) {
            thrTiltCompStrength = 100;
        }
        else if (currentProfile->throttle_tilt_compensation_strength && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
            thrTiltCompStrength = currentProfile->throttle_tilt_compensation_strength;
        }

        if (thrTiltCompStrength) {
            rcCommand[THROTTLE] = constrain(masterConfig.escAndServoConfig.minthrottle
                                            + (rcCommand[THROTTLE] - masterConfig.escAndServoConfig.minthrottle) * calculateThrottleTiltCompensationFactor(thrTiltCompStrength),
                                            masterConfig.escAndServoConfig.minthrottle,
                                            masterConfig.escAndServoConfig.maxthrottle);
        }
    }

    pidController(&currentProfile->pidProfile, currentControlRateProfile, &masterConfig.rxConfig);

#ifdef HIL
    if (hilActive) {
        hilUpdateControlState();
        motorControlEnable = false;
    }
#endif

    mixTable();

#ifdef USE_SERVOS

    if (isMixerUsingServos()) {
        servoMixer(currentProfile->flaperon_throw_offset, currentProfile->flaperon_throw_inverted);
    }

    if (feature(FEATURE_SERVO_TILT)) {
        processServoTilt();
    }

    //Servos should be filtered or written only when mixer is using servos or special feaures are enabled
    if (isServoOutputEnabled()) {
        filterServos();
        writeServos();
    }
#endif

    if (motorControlEnable) {
        writeMotors();
    }

#ifdef USE_SDCARD
        afatfs_poll();
#endif

#ifdef BLACKBOX
    if (!cliMode && feature(FEATURE_BLACKBOX)) {
        handleBlackbox();
    }
#endif
}

// Function for loop trigger
void taskMainPidLoopChecker(void) {
    // getTaskDeltaTime() returns delta time freezed at the moment of entering the scheduler. currentTime is freezed at the very same point.
    // To make busy-waiting timeout work we need to account for time spent within busy-waiting loop
    uint32_t currentDeltaTime = getTaskDeltaTime(TASK_SELF);

    if (masterConfig.gyroSync) {
        while (1) {
            if (gyroSyncCheckUpdate() || ((currentDeltaTime + (micros() - currentTime)) >= (targetLooptime + GYRO_WATCHDOG_DELAY))) {
                break;
            }
        }
    }

    taskMainPidLoop();
}

void taskHandleSerial(void)
{
    handleSerial();
}

void taskUpdateBeeper(void)
{
    beeperUpdate();          //call periodic beeper handler
}

void taskUpdateBattery(void)
{
    static uint32_t vbatLastServiced = 0;
    static uint32_t ibatLastServiced = 0;

    if (feature(FEATURE_VBAT)) {
        if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
            uint32_t vbatTimeDelta = currentTime - vbatLastServiced;
            vbatLastServiced = currentTime;
            updateBattery(vbatTimeDelta);
        }
    }

    if (feature(FEATURE_CURRENT_METER)) {
        int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

        if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
            ibatLastServiced = currentTime;
            updateCurrentMeter(ibatTimeSinceLastServiced, &masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
        }
    }
}

bool taskUpdateRxCheck(uint32_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);
    updateRx(currentTime);
    return shouldProcessRx(currentTime);
}

void taskUpdateRxMain(void)
{
    processRx();
    updatePIDCoefficients(&currentProfile->pidProfile, currentControlRateProfile, &masterConfig.rxConfig);
    isRXDataNew = true;
}

#ifdef GPS
void taskProcessGPS(void)
{
    // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
    // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
    // change this based on available hardware
    if (feature(FEATURE_GPS)) {
        gpsThread();
    }

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTime);
    }
}
#endif

#ifdef MAG
void taskUpdateCompass(void)
{
    if (sensors(SENSOR_MAG)) {
        updateCompass(&masterConfig.magZero);
    }
}
#endif

#ifdef BARO
void taskUpdateBaro(void)
{
    if (sensors(SENSOR_BARO)) {
        uint32_t newDeadline = baroUpdate();
        rescheduleTask(TASK_SELF, newDeadline);
    }

    //updatePositionEstimator_BaroTopic(currentTime);
}
#endif

#ifdef SONAR
void taskUpdateSonar(void)
{
    if (sensors(SENSOR_SONAR)) {
        rangefinderUpdate();
    }

    //updatePositionEstimator_SonarTopic(currentTime);
}
#endif

#ifdef DISPLAY
void taskUpdateDisplay(void)
{
    if (feature(FEATURE_DISPLAY)) {
        updateDisplay();
    }
}
#endif

#ifdef TELEMETRY
void taskTelemetry(void)
{
    telemetryCheckState();

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        telemetryProcess(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);
    }
}
#endif

#ifdef LED_STRIP
void taskLedStrip(void)
{
    if (feature(FEATURE_LED_STRIP)) {
        updateLedStrip();
    }
}
#endif

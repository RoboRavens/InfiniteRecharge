/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.ravenhardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Calibrations;
import frc.ravenhardware.BlinkinCalibrations;

/**
 * "ButtonCode" but for colors of LEDs, mixed with acutal methods for setting
 * the color.
 */
public class RavenBlinkin {
    
    private static Spark _blinkin = new Spark(Calibrations.BLINKIN_PWM);
    private Timer ledDelayer = new Timer();
    private RavenBlinkinPatternCodes nextPatternState;

    // Non-color setters

    private boolean isDelayOver() {        
        ledDelayer.start();
        if (ledDelayer.get() >= BlinkinCalibrations.DELAY_TIME) {
            ledDelayer.stop();
            ledDelayer.reset();
            return true;
        }
        return false;
    }

    // Color methods

    public void blinkGreen() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_GREEN)) {

                solidGreen();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {

                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_GREEN;

            }
        }
    }

    public void solidGreen() {
        _blinkin.set(BlinkinCalibrations.SOLID_GREEN);
    }

    public void blinkYellow() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_YELLOW)) {

                solidYellow();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {

                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_YELLOW;

            }
        }
    }

    public void solidYellow() {
        _blinkin.set(BlinkinCalibrations.SOLID_YELLOW);
    }

    public void blinkRed() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_RED)) {

                solidRed();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {

                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_RED;

            }
        }
    }

    public void solidRed() {
        _blinkin.set(BlinkinCalibrations.SOLID_RED);
    }

    public void blinkBlue() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_BLUE)) {

                solidBlue();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {

                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_BLUE;

            }
        }
    }

    public void solidBlue() {
        _blinkin.set(BlinkinCalibrations.SOLID_BLUE);
    }

    public void solidOff() {
        _blinkin.set(BlinkinCalibrations.LED_OFF);
    }

    public void solidWhite() {
        _blinkin.set(BlinkinCalibrations.SOLID_WHITE);
    }

    public void blinkWhite() {
        if (isDelayOver()) {
            if (nextPatternState.equals(RavenBlinkinPatternCodes.SOLID_WHITE)) {

                solidWhite();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;

            } else {

                solidOff();
                nextPatternState = RavenBlinkinPatternCodes.SOLID_WHITE;

            }
        }
    }
}

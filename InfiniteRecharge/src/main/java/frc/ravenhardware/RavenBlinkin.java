/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.ravenhardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Spark;
import frc.ravenhardware.BlinkinCalibrations;
import frc.robot.commands.LED.CheckFlashCommand;

/**
 * "ButtonCode" but for colors of LEDs, mixed with acutal methods for setting the color.
 * 
 * IMPORTANT: ALL COLOR METHODS MUST BE CALLED USING periodic() OR A SIMILARILY FAST METHOD.
 * 
 */
public class RavenBlinkin {

    private static Spark _blinkin;
    private static Timer ledDelayer;
    public static Timer flashTimer;
    private static RavenBlinkinPatternCodes nextPatternState;
    private static CheckFlashCommand CheckFlash;

    // Non-color setters

    private static boolean isDelayOver() {        
        if (ledDelayer.get() == 0.0) {
            ledDelayer.start();
        }
        if (ledDelayer.get() > BlinkinCalibrations.DELAY_TIME) {
            ledDelayer.stop();
            ledDelayer.reset();
            return true;
        }
        return false;
    }

    public boolean isFlashOver() {        
        if (flashTimer.get() == 0.0) {
            flashTimer.start();
        }
        if (flashTimer.get() > BlinkinCalibrations.FLASH_TIME) {
            flashTimer.stop();
            flashTimer.reset();
            return true;
        }
        return false;
    }

    private void initializeBlinkin(int PWM) {
        _blinkin = new Spark(PWM);
        ledDelayer = new Timer();
        nextPatternState = RavenBlinkinPatternCodes.SOLID_OFF;
    }

    public RavenBlinkin(int PWM) {
        initializeBlinkin(PWM);
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

    public static void solidGreen() {
        _blinkin.set(BlinkinCalibrations.SOLID_GREEN);
    }

    public void flashGreen() {
        // erik magic
        CheckFlash = new CheckFlashCommand(RavenBlinkinPatternCodes.FLASH_GREEN, this);
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

    public static void solidYellow() {
        _blinkin.set(BlinkinCalibrations.SOLID_YELLOW);
    }

    public void flashYellow() {
        // erik magic
        CheckFlash = new CheckFlashCommand(RavenBlinkinPatternCodes.FLASH_YELLOW, this);
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

    public static void solidRed() {
        _blinkin.set(BlinkinCalibrations.SOLID_RED);
    }

    public void flashRed() {
        // erik magic
        CheckFlash = new CheckFlashCommand(RavenBlinkinPatternCodes.FLASH_RED, this);
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

    public static void solidBlue() {
        _blinkin.set(BlinkinCalibrations.SOLID_BLUE);
    }

    public void flashBlue() {
        // erik magic
        CheckFlash = new CheckFlashCommand(RavenBlinkinPatternCodes.FLASH_BLUE, this);
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

    public void flashWhite() {
        // eric magic
    }
}

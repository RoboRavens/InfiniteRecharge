/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.ravenhardware;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private static Boolean flashedRed = false;
    private static Boolean flashedYellow = false;
    private static Boolean flashedGreen = false;

    // Non-color setters

    private static boolean isDelayOver() {        
        if (ledDelayer.get() == 0.0) {
            ledDelayer.start();
        }
        if (ledDelayer.get() >= BlinkinCalibrations.DELAY_TIME) {
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
        if (flashTimer.get() >= BlinkinCalibrations.FLASH_TIME) {
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

    public void checkTime(double time) {
        SmartDashboard.putNumber("Match Time (RavenBlinkin)", time);
        if (time >= 60 && !flashedGreen) {
            this.flashGreen();
        } else if (time >= 30 && !flashedYellow) {
            this.flashYellow();
        } else if (time >= 15 && !flashedRed) {
            this.flashRed();
        }
    }

    // Color methods

    public void solidOff() {
        _blinkin.set(BlinkinCalibrations.LED_OFF);
    }

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
        SmartDashboard.putString("Blinkin's Supposed Color", "Green");
    }

    public void flashGreen() {
        // erik magic
        CheckFlash = new CheckFlashCommand(RavenBlinkinPatternCodes.FLASH_GREEN, this);
        CheckFlash.schedule();
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
        SmartDashboard.putString("Blinkin's Supposed Color", "Yellow");
    }

    public void flashYellow() {
        // erik magic
        CheckFlash = new CheckFlashCommand(RavenBlinkinPatternCodes.FLASH_YELLOW, this);
        CheckFlash.schedule();
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
        SmartDashboard.putString("Blinkin's Supposed Color", "Red");
    }

    public void flashRed() {
        // erik magic
        CheckFlash = new CheckFlashCommand(RavenBlinkinPatternCodes.FLASH_RED, this);
        CheckFlash.schedule();
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
        SmartDashboard.putString("Blinkin's Supposed Color", "Blue");
    }

    public void flashBlue() {
        // erik magic
        CheckFlash = new CheckFlashCommand(RavenBlinkinPatternCodes.FLASH_BLUE, this);
        CheckFlash.schedule();
    }

    public void solidOff() {
        _blinkin.set(BlinkinCalibrations.LED_OFF);
        SmartDashboard.putString("Blinkin's Supposed Color", "Off");
    }

    public void solidWhite() {
        _blinkin.set(BlinkinCalibrations.SOLID_WHITE);
        SmartDashboard.putString("Blinkin's Supposed Color", "White");
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
        // erik magic
        CheckFlash = new CheckFlashCommand(RavenBlinkinPatternCodes.FLASH_WHITE, this);
        CheckFlash.schedule();
    }

}

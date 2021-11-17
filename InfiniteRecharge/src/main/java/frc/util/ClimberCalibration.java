/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

public class ClimberCalibration {

    public String name;
    public int targetHeight;
    public double kF;
    public double kP;
    public double kI;
    public double kD;
    public int upperBoundBuffer;
    public int lowerBoundBuffer;
    public double heightReadyTimerDuration;
    public double conveyanceMagnitude;

    public ClimberCalibration(String name, int height, double kF, double kP, double kI, double kD) {
        this.name = name;
        this.targetHeight = height;
        this.kF = kF;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}

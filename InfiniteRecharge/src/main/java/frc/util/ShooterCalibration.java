/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

public class ShooterCalibration {

    public String name;
    public int rpm;
    public double kF;
    public double kP;
    public double kI;
    public double kD;
    public int upperBoundBuffer;
    public int lowerBoundBuffer;

    public ShooterCalibration(String name, int RPM, double kF, double kP, double kI, double kD, int upperBoundBuffer, int lowerBoundBuffer) {
    this.name = name;
    this.rpm = RPM;
    this.kF = kF;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.upperBoundBuffer = upperBoundBuffer;
    this.lowerBoundBuffer = lowerBoundBuffer;
    }
}

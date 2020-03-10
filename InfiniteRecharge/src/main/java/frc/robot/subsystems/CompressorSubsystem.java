  
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
see compressor doc at
http://first.wpi.edu/FRC/roborio/beta/docs/java/edu/wpi/first/wpilibj/Compressor.html
*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {

  Compressor compressor = new Compressor();
  private boolean _isShooting = false;

  @Override
  public void periodic() {
    if (_isShooting == false) {
      if (RobotController.getBatteryVoltage() < 11.0) {
        stop();
      }

      if (RobotController.getBatteryVoltage() > 11.5) {
        start();
      }
    }
  }

  // enable/disable the auto function
  public void start() {
    compressor.start();
  }

  public void stop() {
    compressor.stop();
  }

  // returns electrical current measured in amps
  public double getCurrentAmps() {
    return compressor.getCompressorCurrent();
  }

  public Boolean isOn() {
    return compressor.getPressureSwitchValue();
  }

  public Boolean isOff() {
    return !isOn();
  }

  public void setIsShooting(boolean isShooting) {
    _isShooting = isShooting;
  }
}
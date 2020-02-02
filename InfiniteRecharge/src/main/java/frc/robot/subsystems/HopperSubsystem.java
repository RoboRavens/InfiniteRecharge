/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HopperSubsystem extends SubsystemBase {

  private TalonSRX _hopperMotor;
  
  public HopperSubsystem() {
    this.initialize();
    _hopperMotor = new TalonSRX(RobotMap.hopperMotor);
  }

  public void initialize() {

  }

  public void periodic() {
    
  }

  public void stopHopperMotors() {
    this.setHopperMotors(0, 0);
  }

  public void reverseHopperMotors() {
    this.setHopperMotors(-0.5, 0.5);
  }

  public void shootHopperMotors() {
    this.setHopperMotors(-1, 1);
  }

  public void agitateHopperMotors() {
    //Does nothing yet
  }

  public void setHopperMotors(double magnitude1, double magnitude2) {
    _hopperMotor.set(ControlMode.PercentOutput, magnitude1);
  }
}

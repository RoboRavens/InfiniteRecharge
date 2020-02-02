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
import frc.robot.Calibrations;
import frc.robot.RobotMap;

public class HopperSubsystem extends SubsystemBase {

  private TalonSRX _hopperLeftMotor;
  private TalonSRX _hopperRightMotor;
  
  public HopperSubsystem() {
    this.initialize();
    _hopperLeftMotor = new TalonSRX(RobotMap.hopperLeftMotor);
    _hopperRightMotor = new TalonSRX(RobotMap.hopperRightMotor);
  }

  public void initialize() {

  }

  public void periodic() {
    
  }

  public void stopHopperMotors() {
    this.setHopperMotors(Calibrations.hopperStop, Calibrations.hopperStop);
  }

  public void feedForward() {
    this.setHopperMotors(Calibrations.leftReverse, Calibrations.rightForward);
  }

  public void feedReverse() {
    this.setHopperMotors(Calibrations.leftForward, Calibrations.rightReverse);
  }

  public void fullReverse() {
    this.setHopperMotors(Calibrations.leftReverse, Calibrations.rightReverse);
  }

  public void fullForward() {
    this.setHopperMotors(Calibrations.leftForward, Calibrations.rightForward);
  }

  public void agitateHopperMotors() {
    //Does nothing yet
  }

  public void setHopperMotors(double leftMagnitude, double rightMagnitude) {
    _hopperLeftMotor.set(ControlMode.PercentOutput, leftMagnitude);
    _hopperRightMotor.set(ControlMode.PercentOutput, rightMagnitude);
  }
}

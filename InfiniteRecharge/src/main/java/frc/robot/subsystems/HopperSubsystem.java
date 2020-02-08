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
    _hopperLeftMotor = new TalonSRX(RobotMap.HOPPER_MOTOR_LEFT);
    _hopperRightMotor = new TalonSRX(RobotMap.HOPPER_MOTOR_RIGHT);
  }

  public void initialize() {

  }

  public void periodic() {
    
  }

  public void stopHopperMotors() {
    this.setHopperMotors(Calibrations.HOPPER_STOP, Calibrations.HOPPER_STOP);
  }

  public void feedForward() {
    this.setHopperMotors(Calibrations.HOPPER_LEFT_REVERSE, Calibrations.HOPPER_RIGHT_FORWARD);
  }

  public void feedReverse() {
    this.setHopperMotors(Calibrations.HOPPER_LEFT_FORWARD, Calibrations.HOPPER_RIGHT_REVERSE);
  }

  public void feedFullSpeed() {
    this.setHopperMotors(Calibrations.HOPPER_FEED_FULL_SPEED_LEFT, Calibrations.HOPPER_FEED_FULL_SPEED_RIGHT);
  }

  public void fullReverse() {
    this.setHopperMotors(Calibrations.HOPPER_LEFT_REVERSE, Calibrations.HOPPER_RIGHT_REVERSE);
  }

  public void fullForward() {
    this.setHopperMotors(Calibrations.HOPPER_LEFT_FORWARD, Calibrations.HOPPER_RIGHT_FORWARD);
  }

  public void agitateHopperMotors() {
    //Does nothing yet
  }

  public void setHopperMotors(double leftMagnitude, double rightMagnitude) {
    _hopperLeftMotor.set(ControlMode.PercentOutput, leftMagnitude);
    _hopperRightMotor.set(ControlMode.PercentOutput, rightMagnitude);
  }
}

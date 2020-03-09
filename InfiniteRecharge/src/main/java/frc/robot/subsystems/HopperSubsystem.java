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
import frc.robot.Robot;

public class HopperSubsystem extends SubsystemBase {

  private TalonSRX _hopperMotor;

  public HopperSubsystem() {
    this.initialize();
    _hopperMotor = new TalonSRX(RobotMap.HOPPER_MOTOR);
  }

  public void initialize() {
  }

  public void periodic() {

  }

  public void stopHopperMotor() {
    this.setHopperMotor(Calibrations.HOPPER_STOP, Calibrations.HOPPER_STOP);
  }

  /*
   * public void feedForward() {
   * this.setHopperMotor(Calibrations.HOPPER_LEFT_REVERSE,
   * Calibrations.HOPPER_RIGHT_FORWARD); }
   */

  /*
   * public void feedReverse() {
   * this.setHopperMotor(Calibrations.HOPPER_LEFT_FORWARD,
   * Calibrations.HOPPER_RIGHT_REVERSE); }
   */

  public void feedFullSpeed() {
    this.setHopperMotor(Calibrations.HOPPER_FEED_FULL_SPEED_LEFT, Calibrations.HOPPER_FEED_FULL_SPEED_RIGHT);
  }

  public void fullReverse() {
    this.setHopperMotor(Calibrations.HOPPER_LEFT_REVERSE, Calibrations.HOPPER_RIGHT_REVERSE);
  }

  public void fullForward() {
    this.setHopperMotor(Calibrations.HOPPER_LEFT_FORWARD, Calibrations.HOPPER_RIGHT_FORWARD);
  }

  public void agitateHopperMotors() {
    fullForward();
  }

  public void setHopperMotor(double leftMagnitude, double rightMagnitude) {
    _hopperMotor.set(ControlMode.PercentOutput, leftMagnitude);
  }

  public void defaultCommand() {
    if (Robot.SHOOTER_SUBSYSTEM.readyToShoot()) {
      this.fullForward();
    } else {
      this.stopHopperMotor();
    }
  }
}

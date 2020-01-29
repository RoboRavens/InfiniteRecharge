/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX _intakeMotor;
  private Solenoid _intakeExtend;
  private Solenoid _intakeRetract;

  public IntakeSubsystem() {
    _intakeMotor = new TalonSRX(RobotMap.intakeMotor);
    _intakeExtend = new Solenoid(RobotMap.intakeExtendSolenoid);
    _intakeRetract = new Solenoid(RobotMap.intakeRetractSolenoid);
  }

  public void initialize() {

  }

  public void periodic() {

  }

  public void harvest(double magnitude) {
    this._intakeMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public void stopMotor() {
    this._intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void intakeExtend() {
    this._intakeRetract.set(false);
    this._intakeExtend.set(true);
  }

  public void intakeRetract() {
    this._intakeRetract.set(true);
    this._intakeExtend.set(false);
  }
}

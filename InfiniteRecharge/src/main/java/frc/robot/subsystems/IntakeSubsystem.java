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
import frc.robot.Calibrations;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX _intakeMotor;
  private Solenoid _intakeExtend;
  private Solenoid _intakeRetract;

  public IntakeSubsystem() {
    this.initialize();
    _intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR);
    _intakeExtend = new Solenoid(RobotMap.INTAKE_EXTEND_SOLENOID);
    _intakeRetract = new Solenoid(RobotMap.INTAKE_RETRACT_SOLENOID);
  }

  public void initialize() {
  }

  public void periodic() {

  }

  public void collect() {
    this.runAtPower(Calibrations.INTAKE_COLLECT_POWER_MAGNITUDE);
  }

  public void spit() {
    this.runAtPower(Calibrations.INTAKE_SPIT_POWER_MAGNITUDE * -1);
  }

  public void runAtPower(double magnitude) {
    this._intakeMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public void stop() {
    this.runAtPower(0);
  }

  public void extend() {
    this._intakeRetract.set(false);
    this._intakeExtend.set(true);
  }

  public void retract() {
    this._intakeRetract.set(true);
    this._intakeExtend.set(false);
  }

  public void stopAndRetract() {
    this.stop();
    this.retract();
  }

  public void defaultCommand() {
    this.stop();
  }
}

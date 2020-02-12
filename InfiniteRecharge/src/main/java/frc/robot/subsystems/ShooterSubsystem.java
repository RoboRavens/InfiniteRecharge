/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.controls.ButtonCode;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.TalonSRXConstants;
import frc.util.NetworkTableDiagnostics;

public class ShooterSubsystem extends SubsystemBase {

  private TalonSRX _shooterMotor;
  private TalonSRX _shooterMotor2;
  private Joystick _joystick;

  private double targetVelocity_UnitsPer100ms = 0;
  double velocity;

  public ShooterSubsystem() {
    _shooterMotor = new TalonSRX(RobotMap.SHOOTER_MOTOR_1);
    _shooterMotor2 = new TalonSRX(RobotMap.SHOOTER_MOTOR_2);
    _shooterMotor2.follow(_shooterMotor);

    _shooterMotor.configFactoryDefault();
    _shooterMotor2.configFactoryDefault();

    _joystick = new Joystick(0);

    /* Config the Velocity closed loop gains in slot0 */
    _shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, Calibrations.SHOOTER_KF, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kP(TalonSRXConstants.kPIDLoopIdx, Calibrations.SHOOTER_KP, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kI(TalonSRXConstants.kPIDLoopIdx, Calibrations.SHOOTER_KI, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kD(TalonSRXConstants.kPIDLoopIdx, Calibrations.SHOOTER_KD, TalonSRXConstants.kTimeoutMs);

    _shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, TalonSRXConstants.kPIDLoopIdx,
        TalonSRXConstants.kTimeoutMs);

    _shooterMotor.configNominalOutputForward(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configNominalOutputReverse(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputForward(1, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputReverse(-1, TalonSRXConstants.kTimeoutMs);
    // VERY IMPORTANT, do not change "false"
    _shooterMotor.setSensorPhase(false);

    // Initialize must be at the bottom, with penalty of null pointer errors
    // this.initialize();
  }

  public void initialize() {
    // setDefaultCommand(new ShooterStopCommand());
    NetworkTableDiagnostics.SubsystemNumber("Shooter", "CurrentVelocity", () -> getVelocity());
    NetworkTableDiagnostics.SubsystemNumber("Shooter", "RPM", () -> getRPM());
    System.out.println("ShooterSubsystem Setup!");
  }

  @Override
  public void periodic() {

  }
/*
  public void setVelocityByButton() {
    // ButtonCode.A is used!
    if (Robot.DRIVE_CONTROLLER.getButtonValue(ButtonCode.X)) {
      // If X is pressed, max speed velocity!
      setVelocity(1.0);

    } else if (Robot.DRIVE_CONTROLLER.getButtonValue(ButtonCode.Y)) {
      // If Y is pressed, half speed velocity!
      setVelocity(0.5);

    } else if (Robot.DRIVE_CONTROLLER.getButtonValue(ButtonCode.B)) {
      // If B is pressed, BRAKE!
      setVelocity(0.0);

    }
  }
*/
  //RPS to FPS
  public double RevolutionsToFeet() {
    return getRPS() * Calibrations.WHEEL_CIRCUMFERENCE_FEET;
  }

  public int getRPS() {
    return getRPM() * 60;
  }

  public void setVelocityBySlider() {
    velocity = _joystick.getThrottle();
    setVelocity(velocity);
  }

  public void setVelocityRaw(int velocity) {
    targetVelocity_UnitsPer100ms = 7600 * velocity;
    SmartDashboard.putNumber("Target Velocity", velocity);
    _shooterMotor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  public void setVelocity(double velocity) {
    targetVelocity_UnitsPer100ms = 7600 * velocity;
    SmartDashboard.putNumber("Target Velocity", velocity);
    _shooterMotor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  public void setRPM(double rpm) {
    setVelocity(rpm * Calibrations.RPM_TO_VEL);
  }

  public void stopShooter() {
    this.setVelocity(0);
  }

  public int getVelocity() {
    return _shooterMotor.getSelectedSensorVelocity();
  }

  public int getRPM() {
    return (int) Math.round(this.getVelocity() / Calibrations.VEL_TO_RPM);
  }

  public void defaultCommand() {
    this.setVelocity(0);
  }
}

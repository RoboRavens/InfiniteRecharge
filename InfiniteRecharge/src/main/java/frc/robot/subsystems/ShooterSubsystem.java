/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
  private double targetVelocity_UnitsPer100ms = 0;

  public ShooterSubsystem() {
    _shooterMotor = new TalonSRX(RobotMap.SHOOTER_MOTOR_1);
    var _shooterMotor2 = new VictorSPX(RobotMap.SHOOTER_MOTOR_2);

    _shooterMotor.configFactoryDefault();
    _shooterMotor2.configFactoryDefault();

    _shooterMotor2.follow(_shooterMotor);
    _shooterMotor2.setInverted(true);
    _shooterMotor2.setNeutralMode(NeutralMode.Coast);

    /* Config the Velocity closed loop gains in slot0 */
    _shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, Calibrations.I_SHOOTER_KF, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kP(TalonSRXConstants.kPIDLoopIdx, Calibrations.I_SHOOTER_KP, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kI(TalonSRXConstants.kPIDLoopIdx, Calibrations.I_SHOOTER_KI, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kD(TalonSRXConstants.kPIDLoopIdx, Calibrations.I_SHOOTER_KD, TalonSRXConstants.kTimeoutMs);

    _shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, TalonSRXConstants.kPIDLoopIdx,
        TalonSRXConstants.kTimeoutMs);

    _shooterMotor.configNominalOutputForward(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configNominalOutputReverse(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputForward(1, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputReverse(-1, TalonSRXConstants.kTimeoutMs);
    // VERY IMPORTANT, do not change "false"
    _shooterMotor.setSensorPhase(false);

    _shooterMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void initialize() {
    NetworkTableDiagnostics.SubsystemNumber("Shooter", "CurrentVelocity", () -> getVelocity());
    NetworkTableDiagnostics.SubsystemNumber("Shooter", "RPM", () -> getRPM());
    System.out.println("ShooterSubsystem Setup!");
  }

  @Override
  public void periodic() {
    printShooterSpeeds();
  }

  //RPS to FPS
  public double RevolutionsToFeet() {
    return getRPS() * Calibrations.WHEEL_CIRCUMFERENCE_FEET;
  }

  public int getRPS() {
    return getRPM() * 60;
  }

  public void setVelocityRaw(double velocity) {
    SmartDashboard.putNumber("Target Velocity", velocity);
    _shooterMotor.set(ControlMode.Velocity, velocity);
  }

  public void setVelocity(double velocity) {
    targetVelocity_UnitsPer100ms = 7600 * velocity;
    SmartDashboard.putNumber("Target Velocity", targetVelocity_UnitsPer100ms);
    //printShooterSpeeds();
    _shooterMotor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  public void setRPM(double rpm) {
    setVelocityRaw(rpm * Calibrations.VEL_TO_RPM);
  }

  public void setPidInit() {
    this._shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, Calibrations.I_SHOOTER_KF, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kP(TalonSRXConstants.kPIDLoopIdx, Calibrations.I_SHOOTER_KP, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kI(TalonSRXConstants.kPIDLoopIdx, Calibrations.I_SHOOTER_KI, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kD(TalonSRXConstants.kPIDLoopIdx, Calibrations.I_SHOOTER_KD, TalonSRXConstants.kTimeoutMs);
  }

  public void setPidCloseTrench() {
    this._shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, Calibrations.C_SHOOTER_KF, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kP(TalonSRXConstants.kPIDLoopIdx, Calibrations.C_SHOOTER_KP, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kI(TalonSRXConstants.kPIDLoopIdx, Calibrations.C_SHOOTER_KI, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kD(TalonSRXConstants.kPIDLoopIdx, Calibrations.C_SHOOTER_KD, TalonSRXConstants.kTimeoutMs);
  }

  public void setPidFarTrench() {
    this._shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, Calibrations.F_SHOOTER_KF, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kP(TalonSRXConstants.kPIDLoopIdx, Calibrations.F_SHOOTER_KP, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kI(TalonSRXConstants.kPIDLoopIdx, Calibrations.F_SHOOTER_KI, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kD(TalonSRXConstants.kPIDLoopIdx, Calibrations.F_SHOOTER_KD, TalonSRXConstants.kTimeoutMs);
  }

  public int secsTillRevved() {
    //double lastTimestamp = Timer.getFPGATimestamp();
    return 1;
  }

  public boolean getIsAtRpmRange(double target) {
    // If RPM is within range, output true. Otherwise, output false
    if (Calibrations.TARGET_RPM_BUFFER > Math.abs(getRPM() - target)) {
      return true;
    }
    return false;
  }

  public void stopShooter() {
    this.setRPM(0);
  }

  public void printShooterSpeeds() {
    System.out.println("UnitsPer100ms: " + getVelocity() + ". RPM: " + getRPM());
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

  public boolean getIsInInitiationLineRpmRange() {
    // If RPM is within range, output true. Otherwise, output false
    if (Calibrations.TARGET_RPM_BUFFER > Math.abs(getRPM() - Calibrations.INIT_LINE_RPM)) {
      return true;
    }

    return false;
  }

  public boolean readyToShoot() {
    boolean overrideIsFalse = !Robot.OPERATION_PANEL.getButtonValue(ButtonCode.SHOOTING_MODE_OVERRIDE);
    boolean isAligned = Robot.LIMELIGHT_SUBSYSTEM.isAlignedToTarget();
    boolean bumperHeld = Robot.DRIVE_CONTROLLER.getButtonValue(ButtonCode.LEFTBUMPER);
    boolean isAtRpm = this.getIsInInitiationLineRpmRange();
    return overrideIsFalse && isAligned && bumperHeld && isAtRpm;
  }

  public boolean readyToShootAuto() {
    boolean overrideIsFalse = true;
    boolean isAligned = true;
    boolean bumperHeld = true;
    boolean isAtRpm = this.getIsInInitiationLineRpmRange();
    return overrideIsFalse && isAligned && bumperHeld && isAtRpm;
  }
}

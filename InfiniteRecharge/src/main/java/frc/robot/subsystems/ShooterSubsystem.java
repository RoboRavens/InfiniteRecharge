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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.RobotMap;
import frc.robot.TalonSRXConstants;
import frc.util.PCDashboardDiagnostics;

public class ShooterSubsystem extends SubsystemBase {

  private TalonSRX _shooterMotor;
  private TalonSRX _shooterMotor2;

  public ShooterSubsystem() {
    _shooterMotor = new TalonSRX(RobotMap.shooterMotor);
    _shooterMotor2 = new TalonSRX(RobotMap.shooterMotor2);
    _shooterMotor2.follow(_shooterMotor);

    /* Config the Velocity closed loop gains in slot0 */
    _shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, Calibrations.shooterkF, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kP(TalonSRXConstants.kPIDLoopIdx, Calibrations.shooterkP, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kI(TalonSRXConstants.kPIDLoopIdx, Calibrations.shooterkI, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kD(TalonSRXConstants.kPIDLoopIdx, Calibrations.shooterkD, TalonSRXConstants.kTimeoutMs);

    _shooterMotor.configFactoryDefault();
    _shooterMotor2.configFactoryDefault();

    _shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, TalonSRXConstants.kPIDLoopIdx,
        TalonSRXConstants.kTimeoutMs);

    _shooterMotor.configNominalOutputForward(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configNominalOutputReverse(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputForward(1, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputReverse(-1, TalonSRXConstants.kTimeoutMs);

    _shooterMotor.setSensorPhase(true);
  }

  public void initialize() {

  }

  @Override
  public void periodic() {
    //make sure to do a test print to confirm connection to dashboard
    double targetVelocity = 0;
    try {
      targetVelocity = Double.parseDouble(SmartDashboard.getString("DB/String 0", "0"));
    } catch (NumberFormatException e) {
      System.out.println("DB/String 0 is not a valid double");
    }

    double currentVelocity = _shooterMotor.getSelectedSensorVelocity();
    this.setVelocity(targetVelocity);
    PCDashboardDiagnostics.SubsystemNumber("ShooterSubsystem", "TargetVelocity", targetVelocity);
    PCDashboardDiagnostics.SubsystemNumber("ShooterSubsystem", "CurrentVelocity", currentVelocity);

  }

  public void setVelocity(double velocity) {

    int encoderVelocityInt = _shooterMotor.getSelectedSensorVelocity();

    // Velocity 600 = about 45 RPM (measured empirically)
    // 25 revolutions = 204661 encoder ticks (measured empirically)
    // 1 rev = ~8186 ticks

    // Velocity: measured in "change in native units per 100 ms"

    // 45 rpm = ~368,370 ticks
    // 60 seconds = 600 100ms segments
    // 600 * 600 = 360,000 ticks, so this checks out within range of measurement
    // error
    // 1 rpm = 8186 ticks in 600 time units
    // 1 rpm = 8192 / 600 = 13.64 velocity

    double velToRpm = 8192 / 600;

    /**
     * Convert 500 RPM to units / 100ms. 4096 Units/Rev * 500 RPM / 600 100ms/min in
     * either direction: velocity setpoint is in units/100ms
     */

    double targetVelocity_UnitsPer100ms = 7600 * velocity;
    double currentTargetRpm = targetVelocity_UnitsPer100ms / velToRpm;

    System.out.println("Trgt: " + Math.round(targetVelocity_UnitsPer100ms) + " RPM: " + Math.round(currentTargetRpm)
        + " Vel: " + Math.round(encoderVelocityInt) + " RPM: " + Math.round(encoderVelocityInt / velToRpm));

    _shooterMotor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }
}

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.controls.ButtonCode;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.TalonSRXConstants;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.util.NetworkTableDiagnostics;

public class ShooterSubsystem extends SubsystemBase {

  private TalonSRX _shooterMotor;
  private TalonSRX _shooterMotor2;
  private Joystick _joystick;

  private double targetVelocity_UnitsPer100ms = 0;
  double velocity = _joystick.getThrottle();

  public ShooterSubsystem() {
    this.initialize();
    _shooterMotor = new TalonSRX(RobotMap.shooterMotor);
    _shooterMotor2 = new TalonSRX(RobotMap.shooterMotor2);
    _shooterMotor2.follow(_shooterMotor);

    _shooterMotor.configFactoryDefault();
    _shooterMotor2.configFactoryDefault();

    _joystick = new Joystick(0);

    /* Config the Velocity closed loop gains in slot0 */
    _shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, Calibrations.shooterkF, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kP(TalonSRXConstants.kPIDLoopIdx, Calibrations.shooterkP, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kI(TalonSRXConstants.kPIDLoopIdx, Calibrations.shooterkI, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.config_kD(TalonSRXConstants.kPIDLoopIdx, Calibrations.shooterkD, TalonSRXConstants.kTimeoutMs);

    _shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, TalonSRXConstants.kPIDLoopIdx,
        TalonSRXConstants.kTimeoutMs);

    _shooterMotor.configNominalOutputForward(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configNominalOutputReverse(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputForward(1, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputReverse(-1, TalonSRXConstants.kTimeoutMs);

    _shooterMotor.setSensorPhase(true);
  }

  public void initialize() {
    setDefaultCommand(new ShooterStopCommand());
    NetworkTableDiagnostics.SubsystemNumber("Shooter", "CurrentVelocity", () -> getVelocity());
    NetworkTableDiagnostics.SubsystemNumber("Shooter", "RPM", () -> getRPM());
    System.out.println("ShooterSubsystem Setup!");
  }

  @Override
  public void periodic() {
    setVelocityBySlider();
    outputForVelocity();
  }

  //Able to tell when the robot is revving through rumble. At full rumble, the robot is close to the target vel!
  private void outputForVelocity() {
    if (targetVelocity_UnitsPer100ms == 0) {
      Robot.DRIVE_CONTROLLER.setRumbleOff();

    } else if (Math.abs(targetVelocity_UnitsPer100ms - getVelocity()) < Calibrations.targetRange) {
      Robot.DRIVE_CONTROLLER.setRumbleOn();
      System.out.println("<Shooter> On Target!");

    } else if (getVelocity() - Calibrations.targetRange <= targetVelocity_UnitsPer100ms) {
      Robot.DRIVE_CONTROLLER.setRumbleCustom(1/(targetVelocity_UnitsPer100ms - getVelocity()), 0);
      System.out.println("<Shooter> Below Target...");

    } else if (getVelocity() + Calibrations.targetRange >= targetVelocity_UnitsPer100ms) {
      Robot.DRIVE_CONTROLLER.setRumbleCustom(0, 1/(getVelocity() - targetVelocity_UnitsPer100ms));
      System.out.println("<Shooter> Above Target...");

    }
  }

  public void setVelocityByButton () {
    //ButtonCode.A is used!
    if(Robot.DRIVE_CONTROLLER.getButtonValue(ButtonCode.X)) {
      //If X is pressed, max speed velocity!
      setVelocity(1.0);

    } else if(Robot.DRIVE_CONTROLLER.getButtonValue(ButtonCode.Y)) {
      //If Y is pressed, half speed velocity!      
      setVelocity(0.5);

    } else if(Robot.DRIVE_CONTROLLER.getButtonValue(ButtonCode.B)) {
      //If B is pressed, BRAKE!      
      setVelocity(0.0);

    }
  }

  //RPS to FPS
  public double RevolutionsToFeet() {
    return getRPS() * Calibrations.wheelCircumferenceFeet;
  }

  public int getRPS() {
    return getRPM() * 60;
  }

  public void setVelocityBySlider () {
    setVelocity(velocity);
  }

  public void setVelocity(double velocity) {
    targetVelocity_UnitsPer100ms = 7600 * velocity;
    _shooterMotor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }

  public void setRPM(double rpm) {
    setVelocity(rpm * Calibrations.RPMToVel);
  }

  public void stopShooter() {
    this.setVelocity(0);
  }

  public int getVelocity() {
    return _shooterMotor.getSelectedSensorVelocity();
  }

  public int getRPM() {
    return (int) Math.round(this.getVelocity() / Calibrations.velToRpm);
  }
} 

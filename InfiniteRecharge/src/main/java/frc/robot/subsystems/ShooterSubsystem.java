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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.controls.ButtonCode;
import frc.ravenhardware.RavenBlinkin;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.TalonSRXConstants;
import frc.util.ShooterCalibration;

public class ShooterSubsystem extends SubsystemBase {

  private TalonSRX _shooterMotor;
  private ShooterCalibration _shot = Calibrations.INIT_LINE;
  private RavenBlinkin _blinkin = new RavenBlinkin(0);

  private Timer _rpmReadyTimer = new Timer();
  
  private Timer _timer = new Timer();
  private double _lowestRPM = 0;
  private boolean _wasInRpmRangeLastCycle = false;
  private double _timeWhenNotInRangeDetected;
  private int _ballsShot = -1; // think of this as a y intercept because it has to rev before shooting balls

  public ShooterSubsystem() {
    _shooterMotor = new TalonSRX(RobotMap.SHOOTER_MOTOR_1);
    var _shooterMotor2 = new VictorSPX(RobotMap.SHOOTER_MOTOR_2);

    _shooterMotor.configFactoryDefault();
    _shooterMotor2.configFactoryDefault();

    _shooterMotor2.follow(_shooterMotor);
    _shooterMotor2.setInverted(true);
    _shooterMotor2.setNeutralMode(NeutralMode.Coast);

    _shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, TalonSRXConstants.kPIDLoopIdx,
        TalonSRXConstants.kTimeoutMs);

    _shooterMotor.configNominalOutputForward(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configNominalOutputReverse(0, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputForward(1, TalonSRXConstants.kTimeoutMs);
    _shooterMotor.configPeakOutputReverse(-1, TalonSRXConstants.kTimeoutMs);
    // VERY IMPORTANT, do not change "false"
    _shooterMotor.setSensorPhase(false);

    _shooterMotor.setNeutralMode(NeutralMode.Coast);
    this.setShot(Calibrations.INIT_LINE);
    _timer.start();
    _rpmReadyTimer.start();
  }

  @Override
  public void periodic() {
    this.printShooterSpeeds();
    this.calculateSecondsToRevUpShot();
    SmartDashboard.putNumber("Balls Shot", _ballsShot);
    this.setLEDs();
  }

  public void setShot(ShooterCalibration shot) {
    this._shot = shot;
    this._shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, _shot.kF, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kP(TalonSRXConstants.kPIDLoopIdx, _shot.kP, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kI(TalonSRXConstants.kPIDLoopIdx, _shot.kI, TalonSRXConstants.kTimeoutMs);
    this._shooterMotor.config_kD(TalonSRXConstants.kPIDLoopIdx, _shot.kD, TalonSRXConstants.kTimeoutMs);

    Robot.CONVEYANCE_SUBSYSTEM.setSynchronizedFeedPowerMagnitude(_shot.conveyanceMagnitude);
  }

  // RPS to FPS
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

  /*
   * public void setVelocity(double velocity) { targetVelocity_UnitsPer100ms =
   * 7600 * velocity; SmartDashboard.putNumber("Target Velocity",
   * targetVelocity_UnitsPer100ms); //printShooterSpeeds();
   * _shooterMotor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms); }
   */

  public void rev() {
    setVelocityRaw(this._shot.targetRpm * Calibrations.VEL_TO_RPM);
  }

  public void stopShooter() {
    this._shooterMotor.set(ControlMode.PercentOutput, 0);
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
    this.stopShooter();
  }

  // If RPM is within range, output true. Otherwise, output false
  public boolean getIsInRpmRange() {
    boolean inRange = true;

    if (this.getRPM() > _shot.targetRpm + _shot.upperBoundBuffer) {
      inRange = false; // rpm is above target + buffer
    }

    if (this.getRPM() < _shot.targetRpm - _shot.lowerBoundBuffer) {
        inRange = false; // rpm is below target - buffer
    }

    // If we are in the RPM range, reset the timer tracking how long it has been since we were in range
    if (inRange) {
        this._rpmReadyTimer.reset();
    }

    // If we were very recently in range, return true, so as to not stop the conveyance.
    if (this._rpmReadyTimer.get() < _shot.rpmReadyTimerDuration) {
        inRange = true;
    }

    return inRange;
  }

  public boolean getIsWideRpmRange() {
    if (this.getRPM() > _shot.targetRpm + Calibrations.YELLOW_RPM_OFFSET) {
      return false; // rpm is above target + buffer
    }

    if (this.getRPM() < _shot.targetRpm - Calibrations.YELLOW_RPM_OFFSET) {
      return false; // rpm is below target - buffer
    }

    return true; // otherwise we are in range
  }

  public void calculateSecondsToRevUpShot() {
    var rpm = this.getRPM();
    if (this.getIsInRpmRange() == false) {
      if (_wasInRpmRangeLastCycle) {
        System.out.println(_timer.get() + " RPM not in range for first time at " + rpm);
        _timeWhenNotInRangeDetected = _timer.get();
      }

      _lowestRPM = Math.min(rpm, _lowestRPM);
      _wasInRpmRangeLastCycle = false;
    } else {
      if (_wasInRpmRangeLastCycle == false) {
        // System.out.println(_timer.get() + " Above target for first time at " + rpm);
        System.out.println("Reached " + _shot.name + " RPM of " + _shot.targetRpm + " from " + _lowestRPM + " after "
            + (_timer.get() - _timeWhenNotInRangeDetected) + " seconds");
        _lowestRPM = this.getRPM();
        _ballsShot++;
      }

      _wasInRpmRangeLastCycle = true;
    }
  }

  public void resetBallsShot() {
    _ballsShot = -1;
  }

  public int getBallsShot() {
    return _ballsShot;
  }

  public boolean readyToShoot() {
    boolean overrideIsFalse = !Robot.OPERATION_PANEL.getButtonValue(ButtonCode.SHOOTING_MODE_OVERRIDE);
    boolean isAligned = Robot.LIMELIGHT_SUBSYSTEM.isAlignedToTarget();
    boolean isAtRpm = this.getIsInRpmRange();
    return overrideIsFalse && isAligned && isAtRpm;
  }

  public boolean readyToShootAuto() {
    boolean overrideIsFalse = true;
    boolean isAligned = true;
    boolean isAtRpm = this.getIsInRpmRange();
    return overrideIsFalse && isAligned && isAtRpm;
  }

  private void setLEDs() {
    if (Robot.OPERATION_PANEL.getButtonValue(ButtonCode.SHOOTERREV)) {
      if (this.getRPM() > _shot.targetRpm + _shot.upperBoundBuffer ) {
        _blinkin.solidBlue();
      }
      else if (this.getIsInRpmRange()) {
        if (Robot.LIMELIGHT_SUBSYSTEM.isAlignedToTarget()) {
          _blinkin.blinkGreen();
        }
        else {
          _blinkin.solidGreen();
        }    
      }
      else if (this.getIsWideRpmRange()) {
        if (Robot.LIMELIGHT_SUBSYSTEM.isAlignedToTarget()) {
          _blinkin.blinkYellow();
        }
        else {
          _blinkin.solidYellow();
        }
      }
      else if (this.getRPM() > 0) {
        if (Robot.LIMELIGHT_SUBSYSTEM.isAlignedToTarget()) {
          _blinkin.blinkRed();
        }
        else {
          _blinkin.solidRed();
        }
      }
      else {
        _blinkin.solidOff();
      }
    }
    else {
      _blinkin.solidOff();
    }
  }
}

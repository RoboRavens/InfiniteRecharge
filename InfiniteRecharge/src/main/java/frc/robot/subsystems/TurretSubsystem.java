/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.RobotMap;;

public class TurretSubsystem extends SubsystemBase {

  private TalonSRX _turretMotor;

  public TurretSubsystem() {
    _turretMotor = new TalonSRX(RobotMap.TURRET_MOTOR);
    
    _turretMotor.configFactoryDefault();
    _turretMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void set(double magnitude) {
    _turretMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public void rotateRight() {
    if (isAtRightLimit()) {
      this.stop();
    }
    else {
    this.set(Calibrations.ROTATION_RIGHT_MAGNITUDE);
    }
  }

  public void rotateLeft() {
    if (isAtLeftLimit()) {
      this.stop();
    }
    else {
    this.set(Calibrations.ROTATION_LEFT_MAGNITUDE);
    }
  }

  public void turretReset() {
    _turretMotor.set(ControlMode.Position, 0);
  }

  public void stop() {
    this.set(0);
  }

  public boolean isAtRightLimit() {
		return _turretMotor.getSensorCollection().isRevLimitSwitchClosed();
	}

	public boolean isAtLeftLimit() {
		return _turretMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }
  
  public void defaultCommand() {
    this.stop();
  }
}


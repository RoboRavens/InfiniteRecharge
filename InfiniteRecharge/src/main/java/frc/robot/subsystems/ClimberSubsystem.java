/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.controls.ButtonCode;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClimberSubsystem extends SubsystemBase {
	private TalonSRX _climberMotor;
	private TalonSRX _climberMotor2;

	public ClimberSubsystem() {

		_climberMotor = new TalonSRX(RobotMap.CLIMBER_MOTOR_1);
		_climberMotor2 = new TalonSRX(RobotMap.CLIMBER_MOTOR_2);
		_climberMotor.configFactoryDefault();
		_climberMotor2.configFactoryDefault();

		// _climberMotor.configForwardLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
		// LimitSwitchNormal.NormallyOpen);
		// _climberMotor.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
		// LimitSwitchNormal.NormallyOpen);
		// _climberMotor2.configForwardLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
		// LimitSwitchNormal.NormallyOpen);
		// _climberMotor2.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX,
		// LimitSwitchNormal.NormallyOpen);

		_climberMotor2.setInverted(true);
	}

	public void extend() {
		if (Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_1)
				&& Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_2)) {
			this.set(Calibrations.CLIMBER_EXTEND_POWER_MAGNITUDE);
		}
	}

	public void retract() {
		if (Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_1)
				&& Robot.OPERATION_PANEL.getButtonValue(ButtonCode.CLIMB_ENABLE_2)) {
			this.set(Calibrations.CLIMBER_RETRACT_POWER_MAGNITUDE);
		}
	}

	private void set(double magnitude) {
		_climberMotor.set(ControlMode.PercentOutput, magnitude);
		_climberMotor2.set(ControlMode.PercentOutput, -1 * magnitude);
	}

	public void periodic() {
		/*
		 * SmartDashboard.putBoolean("Left Climber Is At Extension Limit",
		 * this.leftMotorIsAtExtensionLimit());
		 * SmartDashboard.putBoolean("Left Climber Is At Retraction Limit",
		 * this.leftMotorIsAtRetractionLimit());
		 * SmartDashboard.putBoolean("Right Climber Is At Extension Limit",
		 * this.rightMotorIsAtExtensionLimit());
		 * SmartDashboard.putBoolean("Right Climber Is At Retraction Limit",
		 * this.rightMotorIsAtRetractionLimit());
		 */ }

	public void stop() {
		_climberMotor.set(ControlMode.PercentOutput, 0);
		_climberMotor2.set(ControlMode.PercentOutput, 0);
	}

	public boolean isAtExtensionLimit() {
		return (leftMotorIsAtExtensionLimit() && rightMotorIsAtExtensionLimit());
	}

	public boolean isAtRetractionLimit() {
		return (leftMotorIsAtRetractionLimit() && rightMotorIsAtRetractionLimit());
	}

	private boolean leftMotorIsAtExtensionLimit() {
		return _climberMotor.getSensorCollection().isRevLimitSwitchClosed();
	}

	private boolean rightMotorIsAtExtensionLimit() {
		return _climberMotor2.getSensorCollection().isRevLimitSwitchClosed();
	}

	private boolean leftMotorIsAtRetractionLimit() {
		return _climberMotor.getSensorCollection().isFwdLimitSwitchClosed();
	}

	private boolean rightMotorIsAtRetractionLimit() {
		return _climberMotor2.getSensorCollection().isFwdLimitSwitchClosed();
	}

	public void holdPosition() {
		_climberMotor.set(ControlMode.PercentOutput, Calibrations.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
		_climberMotor2.set(ControlMode.PercentOutput, Calibrations.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
	}

	public void defaultCommand() {
		this.holdPosition();
	}
}
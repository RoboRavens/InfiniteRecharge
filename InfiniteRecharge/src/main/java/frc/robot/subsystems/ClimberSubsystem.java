/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.RobotMap;
import frc.robot.commands.climber.ClimberHoldPositionCommand;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClimberSubsystem extends SubsystemBase {
	private TalonSRX _climberMotor;

	public ClimberSubsystem() {
		/*
		_climberMotor = new TalonSRX(RobotMap.CLIMBER_MOTOR_1);
		var climberMotor2 = new TalonSRX(RobotMap.CLIMBER_MOTOR_2);
		_climberMotor.configFactoryDefault();
		climberMotor2.configFactoryDefault();

		climberMotor2.follow(_climberMotor);
		*/
	}

	public void extend() {
		set(Calibrations.CLIMBER_EXTEND_POWER_MAGNITUDE);
	}

	public void retract() {
		set(Calibrations.CLIMBER_RETRACT_POWER_MAGNITUDE);
	}

	private void set(double magnitude) {
		//_climberMotor.set(ControlMode.PercentOutput, magnitude);
	}

	public void periodic() {

	}

	public void stop() {
		//_climberMotor.set(ControlMode.PercentOutput, 0);
	}

	public boolean isAtExtensionLimit() {
		return false;//_climberMotor.getSensorCollection().isRevLimitSwitchClosed();
	}

	public boolean isAtRetractionLimit() {
		return false;//_climberMotor.getSensorCollection().isFwdLimitSwitchClosed();
	}

	public void holdPosition() {
		//_climberMotor.set(ControlMode.PercentOutput, Calibrations.CLIMBER_HOLD_POSITION_POWER_MAGNITUDE);
	}

	public void defaultCommand() {
		//setDefaultCommand(new ClimberHoldPositionCommand());
	}
}
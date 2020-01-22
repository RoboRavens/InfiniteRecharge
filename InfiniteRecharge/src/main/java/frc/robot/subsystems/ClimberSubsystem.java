/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations;
import frc.robot.RobotMap;
import frc.util.NetworkTableDiagnostics;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;

public class ClimberSubsystem extends SubsystemBase {
	private TalonSRX _climberMotor;
	private Timer _safetyTimer = new Timer();
	private double _expectedPower;

	public ClimberSubsystem() {
		initialize();
		_climberMotor = new TalonSRX(RobotMap.climberMotor);
	}

	public void initialize() {
		// setDefaultCommand(new ClimberHoldPositionCommand());
		NetworkTableDiagnostics.SubsystemNumber("Climber", "Encoder", () -> getEncoderPosition());
		NetworkTableDiagnostics.SubsystemBoolean("Climber", "LimitEncoderExtended", () -> isEncoderAtExtensionLimit());
		NetworkTableDiagnostics.SubsystemBoolean("Climber", "LimitEncoderRetracted", () -> isEncoderAtRetractionLimit());

		// Measure power sent to climber
		NetworkTableDiagnostics.SubsystemNumber("Climber", "EncoderExpectedPower", () -> _expectedPower);
	}

	public void extend(double magnitude) {
		set(-1 * magnitude);
	}

	public void retract(double magnitude) {
		set(magnitude);
	}

	private void set(double magnitude) {
		magnitude = Math.min(magnitude, 1);
		magnitude = Math.max(magnitude, -1);
		magnitude *= 1;

		_expectedPower = magnitude;

		_climberMotor.set(ControlMode.PercentOutput, magnitude);
	}

	public void getPosition() {
		System.out.print("Climber Position: " + getEncoderPosition());
	}

	public double getEncoderPosition() {
		return _climberMotor.getSelectedSensorPosition();
	}

	public void periodic() {
		this.isAtExtensionLimit();
		this.isAtRetractionLimit();

		checkExpectedSpeedVersusPower();
	}

	public void checkExpectedSpeedVersusPower() {
		// Check if climber is being sent power and not moving at the right speed
		if (Math.abs(_expectedPower) > Calibrations.climberHoldPositionPowerMagnitude) {
			// The line below only returns as true if the climber is pushing harder than it needs to not move it
			if (Math.abs(
					_climberMotor.getSelectedSensorVelocity()) < Calibrations.climberConsideredMovingEncoderRate) {
				//burnoutProtection();
			}
		}
	}

	/*public void burnoutProtection() {
		ClimberHoldPositionCommand command = new ClimberHoldPositionCommand();
		command.start();
		command.close();
	}*/

	public void getIsAtLimits() {
		System.out.print(" Extension Limit: " + isAtExtensionLimit() + " Retraction Limit: " + isAtRetractionLimit());
	}

	public void resetEncodersToRetractedLimit() {
		_climberMotor.setSelectedSensorPosition(Calibrations.climberEncoderMinimumValue, 0, 0);
	}

	public void resetEncodersToExtendedLimit() {
		_climberMotor.setSelectedSensorPosition(Calibrations.climberEncoderMaximumValue, 0, 0);
	}

	public void stop() {
		_climberMotor.set(ControlMode.PercentOutput, 0);
	}

	// Right now this method just looks at the right limit switch; some combination of both should be used.

	public boolean isEncoderAtExtensionLimit() {
    	boolean encoderLimit = false;
    	
    	if (this.getEncoderPosition() >= Calibrations.climberEncoderMaximumValue - Calibrations.climberLiftUpwardSafetyMargin) {
    		encoderLimit = true;
    	}
    	
    	return encoderLimit;
    }
    
    public boolean isEncoderAtRetractionLimit() {
    	boolean encoderLimit = false;
    	
    	if (this.getEncoderPosition() <= Calibrations.climberEncoderMinimumValue + Calibrations.climberLiftDownwardSafetyMargin) {
    		encoderLimit = true;
    	}
    	
    	return encoderLimit;
    }

	// Right now this method just looks at the right limit switch; some combination of both should be used.
	public boolean isAtExtensionLimit() {
		return this.isEncoderAtExtensionLimit();
	}

	public boolean isAtRetractionLimit() {
    	return this.isEncoderAtRetractionLimit();
    }

	public void holdPosition() {
		_climberMotor.set(ControlMode.PercentOutput, Calibrations.climberHoldPositionPowerMagnitude);
	}

	public double getClimberHeightPercentage() {
		double encoderMax = (double) Calibrations.climberEncoderMaximumValue;
		double encoderMin = (double) Calibrations.climberEncoderMinimumValue;
		double encoderCurrent = getEncoderPosition();

		double heightPercentage = (encoderCurrent - encoderMin) / (encoderMax - encoderMin);
		heightPercentage = Math.min(1, heightPercentage);
		heightPercentage = Math.max(0, heightPercentage);

		return heightPercentage;
	}

	public static double inchesToTicks(double inches) {
		double encoderTicks = inches;
		encoderTicks -= Calibrations.climberInchesToEncoderTicksOffsetValue;
		encoderTicks *= Calibrations.climberInchesToEncoderTicksConversionValue;

		return encoderTicks;
	}

	public static double ticksToInches(double encoderTicks) {
		double inches = encoderTicks;
		inches /= Calibrations.climberInchesToEncoderTicksConversionValue;
		inches += Calibrations.climberInchesToEncoderTicksOffsetValue;

		return inches;
	}

	public boolean getIsExtendedPastEncoderPosition(int encoderPosition) {
		if (getEncoderPosition() > encoderPosition + Calibrations.CLIMBER_AT_POSITION_BUFFER) {
			return true;
		}

		return false;
	}

	public boolean getIsRetractedBeforeEncoderPosition(int encoderPosition) {
		if (getEncoderPosition() < encoderPosition - Calibrations.CLIMBER_AT_POSITION_BUFFER) {
			return true;
		}

		return false;
	}

	public boolean getIsAtPosition(int encoderPosition) {

		boolean notOverExtended = getIsExtendedPastEncoderPosition(encoderPosition);
		boolean notOverRetracted = getIsRetractedBeforeEncoderPosition(encoderPosition);

		if (notOverExtended == false && notOverRetracted == false) {
			return true;
		}

		return false;
	}

	public void resetSafetyTimer() {
		_safetyTimer.reset();
	}

	public void startSafetyTimer() {
		_safetyTimer.start();
	}

	public double getSafetyTimer() {
		return _safetyTimer.get();
	}

}
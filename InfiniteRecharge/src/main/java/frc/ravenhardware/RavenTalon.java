package frc.ravenhardware;

import frc.robot.Calibrations;
import frc.util.PCDashboardDiagnostics;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Talon;

public class RavenTalon {

	private Talon _talon;
	private TalonFX _talonFX;
	private TalonFX _talonFX2;
	protected double outputSpeed;
	private String _name;
	private double _maxPower;

	// The default slew rate of 2 means no acceleration cutting will occur,
	// as this enables changing between -1 and 1 in a single tick.
	protected double maxSlewRate = 2;

	protected double deadband = .0;

	public RavenTalon(int channel, int channel2, String name, double slewRate) {
		if (Calibrations.USE_TALON_SRX_FOR_DRIVE_CONTROLLER) {
			_talonFX = new TalonFX(channel);
			_talonFX2 = new TalonFX(channel);
			_talonFX.setSensorPhase(false);
			_talonFX2.setSensorPhase(false);
		} else {
			_talon = new Talon(channel);
		}

		_name = name;
		setSlewRate(slewRate);
	}

	// For now, the slew rate is defined in "magnitude of change to
	// motor output, on a -1 to 1 scale, per 'control system tick'" (50hz.)
	// Protip: this number should be greater than zero, but likely not by much.
	// If it's zero the motor will never change output.
	public void setSlewRate(double slewRate) {
		this.maxSlewRate = slewRate;
	}

	public void setMaxPower(double newMaxPower) {
		_maxPower = newMaxPower;
	}

	public void set(double targetOutput) {
		// prevent targetOutput from being greater than maxPower
		if (Math.abs(targetOutput) > _maxPower) {
			targetOutput = Math.signum(targetOutput) * _maxPower;
		}

		// apply deadband to compensate for controller joystick not returning to exactly 0
		if (Math.abs(targetOutput) < this.deadband) {
			targetOutput = 0;
		}

		//Robot.LOGGER_OVERLORD.log(LoggerOverlordLogID.DriveTargetOutputPower, "target output power " + targetOutput);
		this.setWithSlewRate(targetOutput);
	}

	public void setWithSlewRate(double targetOutput) {
		double newOutputSpeed = outputSpeed;

		// Never change the speed by more than the difference between target and actual,
		// regardless of what the slew rate is.
		double slewRate = Math.min(maxSlewRate, Math.abs(targetOutput - outputSpeed));

		// Increment or decrement the new output speed,
		// but never to a magnitude larger than 1.
		if (targetOutput > outputSpeed) {
			newOutputSpeed = outputSpeed + slewRate;

			newOutputSpeed = Math.min(newOutputSpeed, 1);
		}

		if (targetOutput < outputSpeed) {
			newOutputSpeed = outputSpeed - slewRate;

			newOutputSpeed = Math.max(newOutputSpeed, -1);
		}

		// Update and set the output speed.
		outputSpeed = newOutputSpeed;

		PCDashboardDiagnostics.SubsystemNumber("DriveTrain", _name + "OutputPercent", outputSpeed);

		try {
			_talonFX.set(ControlMode.PercentOutput, outputSpeed);
			_talonFX2.set(ControlMode.PercentOutput, outputSpeed);
		} 
		catch (NullPointerException exception) {
			_talon.set(outputSpeed);
		}
	}

	public int getEncoderPosition() {
		try {
			return _talonFX.getSelectedSensorPosition();
		} catch (NullPointerException exception) {
			return 0;
		}
	}

	public void resetEncoderPosition() {
		try {
			_talonFX.setSelectedSensorPosition(0);
			_talonFX2.setSelectedSensorPosition(0);
		} catch (NullPointerException exception) {}
	}
}


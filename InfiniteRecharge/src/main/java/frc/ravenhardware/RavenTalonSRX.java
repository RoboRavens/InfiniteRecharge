package frc.ravenhardware;

import frc.util.PCDashboardDiagnostics;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RavenTalonSRX implements IRavenTalon {
	private TalonSRX _talonSRX;
	protected double outputSpeed;
	private String _name;
	private double _maxPower;
	private boolean _encoderReversed;

	// The default slew rate of 2 means no acceleration cutting will occur,
	// as this enables changing between -1 and 1 in a single tick.
	protected double maxSlewRate = 2;

	protected double deadband = .0;

	public RavenTalonSRX(int mainChannel, int followerChannel, String name, double slewRate, boolean encoderReversed) {
		_talonSRX = new TalonSRX(mainChannel);
		_encoderReversed = encoderReversed;
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

		_talonSRX.set(ControlMode.PercentOutput, outputSpeed);
	}

	public int getEncoderPosition() {
		//what should SRX do?
		return 0;
	}

	public void resetEncoderPosition() {
		// what should the SRX do?
	}

	public void setVoltage(double voltage) {
		// what should the SRX do?
	}

	  /**
   * Get the distance the robot has driven since the last reset as scaled by the value from {@link
   * #setDistancePerPulse(double)}.
   *
   * @return The distance driven since the last reset
   */
	public double getDistanceMeters() {
		//what should SRX do?
		return 0.0;
	}

	public double getRateMeters() {
		//what should SRX do?
		return 0.0;
	}

	public void setCurrentLimit(int amps, int timeoutMs) {
		_talonSRX.configPeakCurrentLimit(amps, timeoutMs);
		_talonSRX.enableCurrentLimit(true);
		_talonSRX.configContinuousCurrentLimit(amps, timeoutMs);
	
   	}
	
   public double getOutputCurrent() {
	   return _talonSRX.getSupplyCurrent();
   }
}


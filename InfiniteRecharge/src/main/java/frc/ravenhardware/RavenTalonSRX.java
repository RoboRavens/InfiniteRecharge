package frc.ravenhardware;

import frc.robot.Calibrations;
import frc.util.PCDashboardDiagnostics;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class RavenTalonSRX {
	private WPI_TalonFX _talonFX;
	private WPI_TalonFX _talonFX2;
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
		_talonFX = newdd WPI_TalonFX(mainChannel);
		_talonFX2 = new WPI_TalonFX(followerChannel);
		_talonFX2.follow(_talonFX);
		//_talonSRX = new TalonSRX(0);
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

		_talonFX.set(ControlMode.PercentOutput, outputSpeed);
	}

	public int getEncoderPosition() {
		return (int) _talonFX.getSensorCollection().getIntegratedSensorPosition();
	}

	public void resetEncoderPosition() {
		_talonFX.getSensorCollection().setIntegratedSensorPosition(0, 10);
	}

	public void setVoltage(double voltage) {
		_talonFX.setVoltage(voltage);
		_talonFX2.setVoltage(voltage);
	}

	  /**
   * Get the distance the robot has driven since the last reset as scaled by the value from {@link
   * #setDistancePerPulse(double)}.
   *
   * @return The distance driven since the last reset
   */
	public double getDistanceMeters() {
		// return _talon.getSelectedSensorPosition(Constants.DriveConstants.pidx) * _distancePerPulse;
		return _talonFX.getSensorCollection().getIntegratedSensorPosition() * Calibrations.ENCODER_DISTANCE_PER_PULSE * (_encoderReversed ? -1 : 1);
	}

	public double getRateMeters() {
		return _talonFX.getSensorCollection().getIntegratedSensorVelocity() * Calibrations.ENCODER_DISTANCE_PER_PULSE * 10 * (_encoderReversed ? -1 : 1);
	}

	/*public void configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg, int timeoutMs) {
		_talonFX.configSupplyCurrentLimit(currLimitCfg, timeoutMs);
	}*/

	public void configStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitCfg, int timeoutMs) {
		_talonFX.configStatorCurrentLimit(currLimitCfg, timeoutMs);
	}

	//i is the amount of amps and j is the timeout in ms
	public void configPeakCurrentLimit(int i, int j) {
		_talonSRX.configPeakCurrentLimit(i, j);
   }

   //i is the amount of amps and j is the timeout in ms
   public void configPeakCurrentDuration(int i, int j) {
	   _talonSRX.configPeakCurrentDuration(i, j);
   }

   //i is the amount of amps and j is the timeout in ms
   public void configContinuousCurrentLimit(int i, int j) {
	   _talonSRX.configContinuousCurrentLimit(i, j);
   }

   public void enableCurrentLimit(boolean b) {
	   _talonSRX.enableCurrentLimit(b);
   }

   public double getOutputCurrent() {
	   return _talonSRX.getSupplyCurrent();
   }
}

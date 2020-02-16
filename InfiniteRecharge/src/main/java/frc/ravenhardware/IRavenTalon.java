package frc.ravenhardware;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public interface IRavenTalon {
	public void setSlewRate(double slewRate);
	public void setMaxPower(double newMaxPower);
	public void set(double targetOutput);
	public void setWithSlewRate(double targetOutput);
	public int getEncoderPosition() ;
	public void resetEncoderPosition() ;
	public void setVoltage(double voltage) ;
	
/*** 
   * Get the distance the robot has driven since the last reset as scaled by the value from {@link
   * #setDistancePerPulse(double)}.
   *
   * @return The distance driven since the last reset
   */
	public double getDistanceMeters();
	public double getRateMeters();

	/*public void configSupplyCurrentLimit(SupplyCurrentLimitConfiguration currLimitCfg, int timeoutMs) {
		_talonFX.configSupplyCurrentLimit(currLimitCfg, timeoutMs);
	}*/

	public void configStatorCurrentLimit(StatorCurrentLimitConfiguration currLimitCfg, int timeoutMs);
	//i is the amount of amps and j is the timeout in ms
	public void configPeakCurrentLimit(int i, int j);
	public void configPeakCurrentDuration(int i, int j);

	public void configContinuousCurrentLimit(int i, int j);
   	public void enableCurrentLimit(boolean b);
    public double getOutputCurrent();
}
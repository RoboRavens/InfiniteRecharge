package frc.ravenhardware;

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

	public void setCurrentLimit(int amps, int timeoutMs);
    public double getOutputCurrent();
}
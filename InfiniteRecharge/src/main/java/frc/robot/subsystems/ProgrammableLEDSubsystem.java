package frc.robot.subsystems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.Color;

import com.ctre.phoenix.CANifier;

public class ProgrammableLEDSubsystem extends SubsystemBase {
	private static CANifier _canifier;
	DriverStation driverStation = DriverStation.getInstance();
	private boolean _teleop = false;

	public ProgrammableLEDSubsystem() {
		this.initialize();
		_canifier = new CANifier(0);
	}

	public void initialize() {
	}

	public void setColorToTeleop() {
		if (Robot.isRedAlliance) {
			this.SetLEDColor(Color.RED);
		} else {
			this.SetLEDColor(Color.BLUE);
		}
	}

	public boolean isTeleopMode() {
		return _teleop;
	}

	public long getMatchSecond() {
		return Math.round((long) driverStation.getMatchTime());
	}

	public boolean getMatchIsAtTime(int desiredMatchSecond) {
		double currentMatchTime = driverStation.getMatchTime();
		if (currentMatchTime > desiredMatchSecond - .5 && currentMatchTime < desiredMatchSecond + .5) {
			return true;
		}
		return false;
	}

	public boolean getMatchIsAtOrAfterTime(int desiredMatchSecond) {
		double currentMatchTime = driverStation.getMatchTime();
		if (currentMatchTime <= desiredMatchSecond) {
			return true;
		}
		return false;
	}

	public boolean getMatchIsAtOrBetweenTimes(int desiredMatchSecond1, int desiredMatchSecond2) {
		double currentMatchTime = driverStation.getMatchTime();
		if (currentMatchTime < desiredMatchSecond1 && currentMatchTime > desiredMatchSecond2) {
			return true;
		}
		return false;
	}

	public void setDisabledMode() {
		_teleop = false;
		this.SetLEDColor(Color.RED);
	}

	public void setAutonomousMode() {
		_teleop = false;
		this.SetLEDColor(Color.WHITE);
	}

	public void setTestMode() {
		_teleop = false;
		this.SetLEDColor(Color.GRAY);
	}

	public void setGamePiecePosessedPattern() {
		this.SetLEDColor(new Color(192, 0, 64));
	}

	public void off() {
		this.SetLEDColor(Color.RED);
	}

	public void SetLEDColor(Color color) {
		float red = (((float) color.getRed() / 256));
		System.out.println(red);
		float green = (((float) color.getGreen() / 256));
		System.out.println(green);
		float blue = (((float) color.getBlue() / 256));
		System.out.println(blue);
		this.SetLEDColor(red, green, blue);
	}

	private void SetLEDColor(float red, float green, float blue) {
		_canifier.setLEDOutput(green, CANifier.LEDChannel.LEDChannelA);
		_canifier.setLEDOutput(red, CANifier.LEDChannel.LEDChannelB);
		_canifier.setLEDOutput(blue, CANifier.LEDChannel.LEDChannelC);
	}
}
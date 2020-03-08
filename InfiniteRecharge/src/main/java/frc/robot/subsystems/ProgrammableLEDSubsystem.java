package frc.robot.subsystems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.Color;

import com.ctre.phoenix.CANifier;

public class ProgrammableLEDSubsystem extends SubsystemBase {
	//private static CANifier _canifier;
	public static Spark _blinkin;
	DriverStation driverStation = DriverStation.getInstance();
	//private boolean _teleop = false;

	public ProgrammableLEDSubsystem() {
		this.initialize();
	}

	public void initialize() {
		_blinkin = new Spark(0);
	}

	public void setRed() {
		_blinkin.set(0.61);
	}

	public void setOff() {
		_blinkin.set(0.99);
	}

	public void setRaw(double color) {
		_blinkin.set(color);
	}

	public void setYellow() {
		_blinkin.set(0.69);
	}

	public void setGreen() {
		_blinkin.set(0.73);
	}

	public void defaultCommand() {
		
	  }
/*
	public void setColorToTeleop() {
		if (Robot.isRedAlliance) {
			this.SetLEDColor(0.05);
		} else {
			this.SetLEDColor(0.80);
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
/*
	public void setDisabledMode() {
		_teleop = false;
		this.SetLEDColor(0.15);
	}

	public void setAutonomousMode() {
		_teleop = false;
		this.SetLEDColor(.65);
	}

	public void setTestMode() {
		_teleop = false;
		this.SetLEDColor(.75);
	}

	public void setGamePiecePosessedPattern() {
		this.SetLEDColor(.85);
	}

	public void off() {
		this.SetLEDColor(0.01);
	}
/*
	public void SetLEDColor(Color color) {
		float red = (((float) color.getRed() / 256));
		System.out.println(red);
		float green = (((float) color.getGreen() / 256));
		System.out.println(green);
		float blue = (((float) color.getBlue() / 256));
		System.out.println(blue);
		this.SetLEDColor(red, green, blue);
	}

	private void SetLEDColor(double color) {
	}
/*
	private void SetLEDColor(float red, float green, float blue) {
		_canifier.setLEDOutput(green, CANifier.LEDChannel.LEDChannelA);
		_canifier.setLEDOutput(red, CANifier.LEDChannel.LEDChannelB);
		_canifier.setLEDOutput(blue, CANifier.LEDChannel.LEDChannelC);
	}*/
}
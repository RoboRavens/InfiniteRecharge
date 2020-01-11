package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OperationPanel {
	private Joystick _joystick;

	public OperationPanel(int port) {
		_joystick = new Joystick(port);
	}

	public boolean getButtonValue(ButtonCode button) {
		return _joystick.getRawButton(getButtonNumber(button));
	}

	public JoystickButton getButton(ButtonCode button) {
		return new JoystickButton(_joystick, getButtonNumber(button));
	}

	public int getButtonNumber(ButtonCode button) {
		int buttonNumber;

		switch (button) {
		case ELEVATORDOUBLEOVERRIDERETRACT:
			buttonNumber = 1;
			break;
		case ELEVATORDOUBLEOVERRIDEEXTEND:
			buttonNumber = 2;
			break;
		case ARMDOUBLEOVERRIDERETRACT:
			buttonNumber = 3;
			break;
		case ARMDOUBLEOVERRIDEEXTEND:
			buttonNumber = 4;
			break;
		case ELEVATOROVERRIDERETRACT:
			buttonNumber = 5;
			break;
		case ELEVATOROVERRIDEEXTEND:
			buttonNumber = 6;
			break;
		case ARMOVERRIDERETRACT:
			buttonNumber = 7;
			break;
		case ARMOVERRIDEEXTEND:
			buttonNumber = 8;
			break;
		case ROCKETHIGH:
			buttonNumber = 9;
			break;
		case ROCKETMID:
			buttonNumber = 10;
			break;
		case ROCKETLOW:
			buttonNumber = 11;
			break;
		case CARGOSHIP:
			buttonNumber = 12;
			break;
		default:
			throw new IllegalArgumentException("Invalid Button Code" + button);
		}

		return buttonNumber;
	}
}

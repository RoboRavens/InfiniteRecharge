package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperationPanel2 {
	private Joystick _joystick;

	public OperationPanel2(int port) {
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
		case SHOOTFURTHER:
			buttonNumber = 2;
			break;
		case HOPPERAGITATE:
			buttonNumber = 3;
			break;
		case SETSHOTFARCONTROLPANEL:
			buttonNumber = 4;
			break;
		case SETSHOTCLOSECONTROLPANEL:
			buttonNumber = 5;
			break;
		case SETSHOTINITIATIONLINE:
			buttonNumber = 6;
			break;
		case CONVEYANCESHOOT:
			buttonNumber = 7;
			break;
		default:
			throw new IllegalArgumentException("Invalid Button Code: " + button);
		}

		return buttonNumber;
	}
}
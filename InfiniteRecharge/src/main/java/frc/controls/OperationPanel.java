package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
		case SHOOTERREV:
			buttonNumber = 2;
			break;
		case OVERRIDEREVERSECONVEYANCE:
			buttonNumber = 5;
			break;
		case READYTOSHOOT:
			buttonNumber = 6;
			break;
		case OVERRIDECLIMBEXTEND:
			buttonNumber = 7;
			break;
		case OVERRIDECLIMBRETRACT:
			buttonNumber = 8;
			break;
		case SETCLIMBERTOPOSITION:
			buttonNumber = 9;
			break;
		case SETCLIMBERTORETRACTED:
			buttonNumber = 11;
			break;
		default:
			throw new IllegalArgumentException("Invalid Button Code" + button);
		}

		return buttonNumber;
	}
}

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
		case SHOOTING_MODE_OVERRIDE:
			buttonNumber = 1;
			break;
		case SHOOTERREV:
			buttonNumber = 2;
			break;
		case CLIMB_ENABLE_1:
			buttonNumber = 3;
			break;
		case CLIMB_ENABLE_2:
			buttonNumber = 4;
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

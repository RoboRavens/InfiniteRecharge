package frc.robot.commands.climber;

import frc.robot.Calibrations;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberExtendFullyCommand extends CommandBase {

	public ClimberExtendFullyCommand() {
		addRequirements(Robot.CLIMBER_SUBSYSTEM);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		System.out.println("ClimberExtendFullyCommand init");
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		if (Robot.CLIMBER_SUBSYSTEM.isAtExtensionLimit() == false) {
			Robot.CLIMBER_SUBSYSTEM.extend(Calibrations.CLIMBER_EXTEND_POWER_MAGNITUDE);
		} else {
			Robot.CLIMBER_SUBSYSTEM.stop();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.CLIMBER_SUBSYSTEM.resetEncodersToExtendedLimit();
		Robot.CLIMBER_SUBSYSTEM.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (Robot.CLIMBER_SUBSYSTEM.isAtExtensionLimit()) {
			return true;
		}

		return false;
	}
}

package frc.robot.commands.climber;

import frc.robot.Calibrations;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberRetractFullyCommand extends CommandBase {

	public ClimberRetractFullyCommand() {
		addRequirements(Robot.CLIMBER_SUBSYSTEM);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		System.out.println("ClimberRetractFullyCommand init");
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		if (Robot.CLIMBER_SUBSYSTEM.isAtRetractionLimit() == false) {
			Robot.CLIMBER_SUBSYSTEM.retract(Calibrations.CLIMBER_RETRACT_POWER_MAGNITUDE);
		} else {
			Robot.CLIMBER_SUBSYSTEM.stop();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		Robot.CLIMBER_SUBSYSTEM.resetEncodersToRetractedLimit();
		Robot.CLIMBER_SUBSYSTEM.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		boolean isFinished = false;

		if (Robot.CLIMBER_SUBSYSTEM.isAtRetractionLimit()) {
			isFinished = true;
		}
		return isFinished;
	}
}

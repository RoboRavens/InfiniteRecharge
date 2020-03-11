package frc.robot.commands.climber;

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
		System.out.println("RETRACTING_CLIMBER_FULLY!!!");
		Robot.CLIMBER_SUBSYSTEM.retract();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (Robot.CLIMBER_SUBSYSTEM.isAtRetractionLimit()) {
			System.out.println("ClimberRetractFullyCommand finished");
			Robot.CLIMBER_SUBSYSTEM.stop();
			return true;
		}

		return false;
	}
}

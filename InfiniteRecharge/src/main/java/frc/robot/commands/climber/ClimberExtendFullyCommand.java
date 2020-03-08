package frc.robot.commands.climber;

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
		System.out.println("EXTENDING_CLIMBER_FULLY!!!");
		Robot.CLIMBER_SUBSYSTEM.extend();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (Robot.CLIMBER_SUBSYSTEM.isAtExtensionLimit()) {
			System.out.println("ClimberExtendFullyCommand finished");
			Robot.CLIMBER_SUBSYSTEM.stop();
			return true;
		}

		return false;
	}
}

package frc.robot.commands.climber;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberHoldPositionCommand extends CommandBase {
	double targetPosition;

	public ClimberHoldPositionCommand() {
		addRequirements(Robot.CLIMBER_SUBSYSTEM);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		System.out.println("ClimberHoldPositionCommand init");
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		// This fights against the constant force springs keeping the climber extended.
		if (Robot.CLIMBER_SUBSYSTEM.isAtExtensionLimit() == false) {
			Robot.CLIMBER_SUBSYSTEM.holdPosition();
		} else {
			Robot.CLIMBER_SUBSYSTEM.stop();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

}

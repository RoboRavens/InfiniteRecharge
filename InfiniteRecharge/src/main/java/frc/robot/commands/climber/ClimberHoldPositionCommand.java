package frc.robot.commands.climber;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ClimberHoldPositionCommand extends Command {
	double targetPosition;

	public ClimberHoldPositionCommand() {
		requires(Robot.CLIMBER_SUBSYSTEM);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("ClimberHoldPositionCommand init");
		this.targetPosition = Robot.CLIMBER_SUBSYSTEM.getEncoderPosition();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		// The goal of this command is to send a very small amount of power to the
		// elevator motors to fight against gravity - NOT to move the elevator, at all.
		if (Robot.CLIMBER_SUBSYSTEM.isAtRetractionLimit() == false) {
			Robot.CLIMBER_SUBSYSTEM.holdPosition();
		} else {
			Robot.CLIMBER_SUBSYSTEM.stop();
		}
	}

	protected boolean isFinished() {
		// This command is never finished; once it is called, the only way it stops is
		// if another command overrides it.
		return false;
	}

}

package frc.robot.commands.climber;

import frc.robot.Calibrations;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ClimberRetractFullyCommand extends Command {

	public ClimberRetractFullyCommand() {
		requires(Robot.CLIMBER_SUBSYSTEM);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		System.out.println("ClimberRetractFullyCommand init");
		Robot.CLIMBER_SUBSYSTEM.resetSafetyTimer();
		Robot.CLIMBER_SUBSYSTEM.startSafetyTimer();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.CLIMBER_SUBSYSTEM.isAtRetractionLimit() == false) {
    		Robot.CLIMBER_SUBSYSTEM.retract(Calibrations.climberRetractPowerMagnitude);
    	}
    	else {
    		Robot.CLIMBER_SUBSYSTEM.stop();
    	}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		boolean isFinished = false;

		if (Robot.CLIMBER_SUBSYSTEM.getSafetyTimer() > Calibrations.CLIMBER_SAFETY_TIMER_TIMEOUT) {
			System.out.println("TIMEOUT TIMEOUT TIMEOUT TIMEOUT");
			isFinished = true;
		}

		if (Robot.CLIMBER_SUBSYSTEM.isAtRetractionLimit()) {
			isFinished = true;
		}
		return isFinished;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.CLIMBER_SUBSYSTEM.resetEncodersToRetractedLimit();;
		Robot.CLIMBER_SUBSYSTEM.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}

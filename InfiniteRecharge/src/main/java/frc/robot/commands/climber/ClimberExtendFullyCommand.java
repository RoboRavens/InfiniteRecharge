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
    		Robot.CLIMBER_SUBSYSTEM.extend(Calibrations.climberExtendPowerMagnitude);
    	}
    	else {
    		Robot.CLIMBER_SUBSYSTEM.stop();
    	}
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		if (Robot.CLIMBER_SUBSYSTEM.isAtExtensionLimit()) {
			return true;
		}

		return false;
	}

	// Called once after isFinished returns true
	public void end() {
		Robot.CLIMBER_SUBSYSTEM.resetEncodersToExtendedLimit();
		Robot.CLIMBER_SUBSYSTEM.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	public void interrupted() {
	}
}

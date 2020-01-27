package frc.robot.commands.drivetrain;

import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetGyroTargetHeadingCommand extends CommandBase {

	private int _heading;
	
    public SetGyroTargetHeadingCommand(int heading) {
    	_heading = heading;
        
    }

    // Called just before this Command runs the first time
    public void initialize() {
        System.out.println("SetGyroTargetHeadingCommand init");
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
    	Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.setGyroTargetHeading(_heading);
    }

     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) {
     }
 
     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
         return true;
     }
}

package frc.robot.commands.drivetrain;

import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainStopCommand extends CommandBase {

    public DriveTrainStopCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    // Called just before this Command runs the first time
    public void initialize() {
        System.out.println("DriveTrainStopCommand init");
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.stop();
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.gyroStop();
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    public void end() {
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.stop();
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.gyroStop();
    }

    // Called when another command which addRequirements one or more of the same
    // subsystems is scheduled to run
    public void interrupted() {
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.stop();
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.gyroStop();
    }
}

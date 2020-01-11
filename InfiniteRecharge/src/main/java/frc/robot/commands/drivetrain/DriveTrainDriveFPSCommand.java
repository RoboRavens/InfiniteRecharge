package frc.robot.commands.drivetrain;

import frc.controls.AxisCode;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainDriveFPSCommand extends CommandBase {

    public DriveTrainDriveFPSCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    // Called just before this Command runs the first time
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        double leftYAxisValue = Robot.DRIVE_CONTROLLER.getAxis(AxisCode.LEFTSTICKY);
        double rightYAxisValue = Robot.DRIVE_CONTROLLER.getAxis(AxisCode.RIGHTSTICKY);
        double rightXAxisValue = Robot.DRIVE_CONTROLLER.getAxis(AxisCode.RIGHTSTICKX);
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.drive(leftYAxisValue, rightYAxisValue, rightXAxisValue);
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    public void end() {
    }

    // Called when another command which addRequirements one or more of the same
    // subsystems is scheduled to run
    public void interrupted() {
    }
}

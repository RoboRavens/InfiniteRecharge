package frc.robot.commands.drivetrain;

import frc.controls.AxisCode;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainDriveLimeLightCommand extends CommandBase {

    public DriveTrainDriveLimeLightCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    // Called just before this Command runs the first time
    public void initialize() {
        System.out.println("DriveTrainDriveLimeLightCommand init");
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        double leftYAxisValue = Robot.DRIVE_CONTROLLER.getAxis(AxisCode.LEFTSTICKY);
        double rightXAxisValue = Robot.DRIVE_CONTROLLER.getAxis(AxisCode.RIGHTSTICKX);
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.fpsTankChooseLimelightOrManual(leftYAxisValue, rightXAxisValue);
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

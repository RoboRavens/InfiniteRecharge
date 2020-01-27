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

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.stop();
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.gyroStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}

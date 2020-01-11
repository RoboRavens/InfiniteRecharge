package frc.robot.commands.drivetrain;

import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetCutPowerFalseCommand extends CommandBase {

    public SetCutPowerFalseCommand() {
        addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    }

    // Called just before this Command runs the first time
    public void initialize() {
        System.out.println("SetCutPowerFalseCommand init");
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.setCutPower(false);
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    public void end() {
    }

    // Called when another command which addRequirements one or more of the same
    // subsystems is scheduled to run
    public void interrupted() {
    }
}

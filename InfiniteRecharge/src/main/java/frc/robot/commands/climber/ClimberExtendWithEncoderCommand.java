package frc.robot.commands.climber;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberExtendWithEncoderCommand extends CommandBase {

  public ClimberExtendWithEncoderCommand() {
        addRequirements(Robot.CLIMBER_SUBSYSTEM);
    }

    // Called just before this Command runs the first time
    public void initialize() {
        //System.out.println("ClimberExtendWithEncodderCommand init");
        Robot.CLIMBER_SUBSYSTEM.setTargetExtended();
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        Robot.CLIMBER_SUBSYSTEM.setBasedOnTarget();
        //System.out.println("EXTENDING_CLIMBER_WITH_ENDCODER_HELD!!!");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.CLIMBER_SUBSYSTEM.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

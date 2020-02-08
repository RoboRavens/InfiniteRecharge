package frc.robot.commands.climber;

import frc.robot.Calibrations;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberRetractWhileHeldCommand extends CommandBase {

    public ClimberRetractWhileHeldCommand() {
        addRequirements(Robot.CLIMBER_SUBSYSTEM);

    }

    // Called just before this Command runs the first time
    public void initialize() {
        System.out.println("ClimberRetractWhileHeldCommand init");
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        Robot.CLIMBER_SUBSYSTEM.retract(Calibrations.CLIMBER_RETRACT_POWER_MAGNITUDE);
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

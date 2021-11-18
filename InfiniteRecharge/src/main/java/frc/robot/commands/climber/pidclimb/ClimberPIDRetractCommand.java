package frc.robot.commands.climber.pidclimb;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations;

public class ClimberPIDRetractCommand extends CommandBase {
    
    public ClimberPIDRetractCommand() {
        addRequirements(Robot.CLIMBER_SUBSYSTEM);
    }

    public void initialize() {

    }

    public void execute() {
        Robot.CLIMBER_SUBSYSTEM.setPID(Calibrations.LO_CLIMB);
        Robot.CLIMBER_SUBSYSTEM.elevate();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
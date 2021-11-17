package frc.robot.commands.climber.pidclimb;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations;

public class ClimberPIDExtendCommand extends CommandBase {
    
    public ClimberPIDExtendCommand() {
        addRequirements(Robot.CLIMBER_SUBSYSTEM);
    }

    public void initialize() {

    }

    public void execute() {
        Robot.CLIMBER_SUBSYSTEM.setPIDHeight(Calibrations.HI_CLIMB);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
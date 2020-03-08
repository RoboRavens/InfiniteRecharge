/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ClimberReleaseHooksCommand extends CommandBase {
  
  public ClimberReleaseHooksCommand() {
    addRequirements(Robot.CLIMBER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ClimberReleaseHooksCommand init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("CLIMBER_RELEASING_HOOKS");
    Robot.CLIMBER_SUBSYSTEM.extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.CLIMBER_SUBSYSTEM.isAtHookReleaseLimit()) {
			System.out.println("ClimberReleaseHooksCommand finished");
			Robot.CLIMBER_SUBSYSTEM.stop();
			return true;
		}
		return false;
  }
}

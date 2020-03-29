/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Calibrations;

public class TurretTurnToTargetCommand extends CommandBase {

  public TurretTurnToTargetCommand() {
    addRequirements(Robot.TURRET_SUBSYSTEM, Robot.LIMELIGHT_SUBSYSTEM);
  }

  // Called just before this Command runs the first time
  public void initialize() {
    System.out.println("TurretTurnToTargetCommand init");
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    Robot.LIMELIGHT_SUBSYSTEM.turnToTargetTurret();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = false;
    if (Robot.LIMELIGHT_SUBSYSTEM.angleOffHorizontal() <= Calibrations.DESIRED_TURRET_TARGET_BUFFER) {
      isFinished = true;
    }
    return isFinished;
  }
}

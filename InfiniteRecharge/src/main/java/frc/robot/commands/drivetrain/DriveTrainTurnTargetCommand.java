/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveTrainTurnTargetCommand extends CommandBase {

  public DriveTrainTurnTargetCommand() {
    addRequirements(Robot.LIMELIGHT_SUBSYSTEM);
  }

  // Called just before this Command runs the first time
  public void initialize() {
    System.out.println("DriveTrainTurnTargetCommand init");
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    Robot.LIMELIGHT_SUBSYSTEM.turnToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = false;
    if (Robot.LIMELIGHT_SUBSYSTEM.angleOffHorizontal() <= 5) {
      isFinished = true;
    }
    return isFinished;
  }
}

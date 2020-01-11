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
    System.out.println("DRIVETRAINTURNTARGETCOMMANDISRUNNING");
    Robot.LIMELIGHT_SUBSYSTEM.turnToTarget();
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  public void end() {
  }

  // Called when another command which addRequirements one or more of the same
  // subsystems is scheduled to run
  public void interrupted() {
  }
}

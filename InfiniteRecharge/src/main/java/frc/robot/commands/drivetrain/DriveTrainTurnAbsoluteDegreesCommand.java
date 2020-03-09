/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * Useful for fixing the angle of the robot after a path completes.
 */
public class DriveTrainTurnAbsoluteDegreesCommand extends CommandBase {
  double _previousGyroScaleFactor;

  public DriveTrainTurnAbsoluteDegreesCommand(double targetDegrees) {
    addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
    // TODO: finish command
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

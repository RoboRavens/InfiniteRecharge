/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.DriveTrainTurnTargetCommand;

public class PrepareShotCommandGroup extends ParallelCommandGroup {

  public PrepareShotCommandGroup() {
    super(new DriveTrainTurnTargetCommand(), new ShooterRevCommand(Robot.SHOOTER_SUBSYSTEM.getTargetRPM()));
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.DriveTrainDriveInchesCommand;
// import frc.robot.commands.drivetrain.DriveTrainDriveInchesCommand;
import frc.robot.commands.shooter.ShooterAutonomousShootCommand;

public class DriveAndShootAutonomousCommand {
  public static Command GenerateCommand() {
    var drive2FeetTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(),
        new Pose2d(24 / Calibrations.METERS_TO_INCHES, 0, new Rotation2d(0)), Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig());

    var drive2FeetCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(drive2FeetTrajectory); 

    return new SequentialCommandGroup(new ShooterAutonomousShootCommand(),
        new DriveTrainDriveInchesCommand(24, .25, Calibrations.DRIVING_FORWARD));
        // drive2FeetCommand
        // );
  }
}

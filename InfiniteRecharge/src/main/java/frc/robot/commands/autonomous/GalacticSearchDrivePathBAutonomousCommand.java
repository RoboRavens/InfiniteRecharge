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
import frc.robot.commands.drivetrain.DriveTrainTurnRelativeDegreesCommand;
import frc.robot.commands.drivetrain.SetGyroTargetHeadingCommand;
// import frc.robot.commands.drivetrain.DriveTrainDriveInchesCommand;
import frc.robot.commands.shooter.ShooterAutonomousShootCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

public class GalacticSearchDrivePathBAutonomousCommand {
  public static Command GenerateCommand() {
    DriveTrainSubsystem drivetrain = Robot.DRIVE_TRAIN_SUBSYSTEM;

    int robotLength = 36;
    int firstEast = 180 - robotLength;
    int diagonal = 85;
    int eastDistance = 150;

    double powerMagnitude = .25;
    

    return new SequentialCommandGroup(
        // Assumes the robot starts at a 45 degree angle facing northeast
        new DriveTrainDriveInchesCommand(firstEast, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new SetGyroTargetHeadingCommand(-45),
        new DriveTrainDriveInchesCommand(diagonal, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new SetGyroTargetHeadingCommand(45),
        new DriveTrainDriveInchesCommand(diagonal * 2, 1, Calibrations.DRIVING_FORWARD)

        //new ShooterAutonomousShootCommand(),
        // new DriveTrainDriveInchesCommand(24, .25, Calibrations.DRIVING_FORWARD));
        //drive2FeetCommand
        );
  }
}

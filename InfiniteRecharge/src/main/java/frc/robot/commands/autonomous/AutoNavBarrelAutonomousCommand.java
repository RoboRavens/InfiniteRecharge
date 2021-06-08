/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.util.List;

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

public class AutoNavBarrelAutonomousCommand {
  public static Command GenerateCommand() {
    DriveTrainSubsystem drivetrain = Robot.DRIVE_TRAIN_SUBSYSTEM;

    int firstForward = 130;
    int squareSide = 45;
    int secondForward = 130;
    int firstNorth = 55;
    int returnSouth = firstNorth + 30;
    int thirdForward = 130;
    int returnHome = firstForward + secondForward + thirdForward + 50;
    double powerMagnitude = .25;

    return new SequentialCommandGroup(
        new DriveTrainDriveInchesCommand(firstForward, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new DriveTrainDriveInchesCommand(squareSide, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new DriveTrainDriveInchesCommand(squareSide, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new DriveTrainDriveInchesCommand(squareSide, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new DriveTrainDriveInchesCommand(secondForward, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(firstNorth, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(squareSide, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(returnSouth, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(thirdForward, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(squareSide + 10, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -95, .015),
        new SetGyroTargetHeadingCommand(180),
        new DriveTrainDriveInchesCommand(returnHome, powerMagnitude, Calibrations.DRIVING_FORWARD)

        //new ShooterAutonomousShootCommand(),
        // new DriveTrainDriveInchesCommand(24, .25, Calibrations.DRIVING_FORWARD));
        //drive2FeetCommand
        );
  }
}

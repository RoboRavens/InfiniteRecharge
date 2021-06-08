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

public class AutoNavSlalomAutonomousCommand {
  public static Command GenerateCommand() {
    DriveTrainSubsystem drivetrain = Robot.DRIVE_TRAIN_SUBSYSTEM;

    int robotLength = 36;
    int squareDiagonal = 85; // + robotLength;
    int firstForward = 120;
    int squareSide = 45;

    double powerMagnitude = .25;

    return new SequentialCommandGroup(
        new DriveTrainDriveInchesCommand(45, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new SetGyroTargetHeadingCommand(-90),
        new DriveTrainDriveInchesCommand(50, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new SetGyroTargetHeadingCommand(0),
        // First straightaway
        new DriveTrainDriveInchesCommand(150, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new SetGyroTargetHeadingCommand(90),

        new DriveTrainDriveInchesCommand(60, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new SetGyroTargetHeadingCommand(0),

        
        // Extend past last cone
        new DriveTrainDriveInchesCommand(60, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new SetGyroTargetHeadingCommand(-90),

        // Switch sides of cones
        new DriveTrainDriveInchesCommand(40, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new SetGyroTargetHeadingCommand(-180),

        // Come back from last cone
        new DriveTrainDriveInchesCommand(55, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new SetGyroTargetHeadingCommand(-270),


        new DriveTrainDriveInchesCommand(50, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new SetGyroTargetHeadingCommand(-180),
        new DriveTrainDriveInchesCommand(155, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new SetGyroTargetHeadingCommand(-90),
        new DriveTrainDriveInchesCommand(50, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new SetGyroTargetHeadingCommand(-180),
        new DriveTrainDriveInchesCommand(90, powerMagnitude, Calibrations.DRIVING_FORWARD)
    /*
        // Assumes the robot starts at a 45 degree angle facing northeast
        new DriveTrainDriveInchesCommand(squareDiagonal, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 45),
        new DriveTrainDriveInchesCommand(firstForward, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 45),
        new DriveTrainDriveInchesCommand(squareDiagonal, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(squareSide, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(squareSide, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(squareSide, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(squareDiagonal, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(firstForward, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(squareDiagonal, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(robotLength, powerMagnitude, Calibrations.DRIVING_FORWARD)
*/
        //new ShooterAutonomousShootCommand(),
        // new DriveTrainDriveInchesCommand(24, .25, Calibrations.DRIVING_FORWARD));
        //drive2FeetCommand
        );
  }
}

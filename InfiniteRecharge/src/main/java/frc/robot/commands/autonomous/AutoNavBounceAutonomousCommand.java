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
// import frc.robot.commands.drivetrain.DriveTrainDriveInchesCommand;
import frc.robot.commands.shooter.ShooterAutonomousShootCommand;

public class AutoNavBounceAutonomousCommand {
  public static Command GenerateCommand() {
    DriveTrainSubsystem drivetrain = Robot.DRIVE_TRAIN_SUBSYSTEM;

    int robotLength = 36;
    int oneUnit = 30; // + robotLength;
    int squareDiagonal = 85;
    double halfSquareDiagonal = squareDiagonal / 2;
    int segmentTwoNorthDistance = 90;
    int finalSouth = 70;
    
    // Inches north of center of the start zone that the robot starts
    int northOffset = 10;
    

    return new SequentialCommandGroup(
        // Assumes the robot starts at a 45 degree angle facing northeast
        new DriveTrainDriveInchesCommand(robotLength + oneUnit, 1, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(oneUnit, 1, Calibrations.DRIVING_FORWARD),
        new DriveTrainDriveInchesCommand(oneUnit + northOffset, 1, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(squareDiagonal, 1, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new DriveTrainDriveInchesCommand(halfSquareDiagonal, 1, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(segmentTwoNorthDistance, 1, Calibrations.DRIVING_FORWARD),
        new DriveTrainDriveInchesCommand(segmentTwoNorthDistance, 1, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(halfSquareDiagonal, 1, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(oneUnit, 1, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(halfSquareDiagonal, 1, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(segmentTwoNorthDistance, 1, Calibrations.DRIVING_BACKWARD),
        new DriveTrainDriveInchesCommand(finalSouth, 1, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(finalSouth, 1, Calibrations.DRIVING_FORWARD)

        //new ShooterAutonomousShootCommand(),
        // new DriveTrainDriveInchesCommand(24, .25, Calibrations.DRIVING_FORWARD));
        //drive2FeetCommand
        );
  }
}

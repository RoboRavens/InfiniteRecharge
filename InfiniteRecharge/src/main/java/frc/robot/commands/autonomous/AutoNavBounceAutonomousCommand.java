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
// import frc.robot.commands.drivetrain.DriveTrainDriveInchesCommand;
import frc.robot.commands.shooter.ShooterAutonomousShootCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoNavBounceAutonomousCommand {
  public static Command GenerateCommand() {
    DriveTrainSubsystem drivetrain = Robot.DRIVE_TRAIN_SUBSYSTEM;

    int robotLength = 36;
    int oneUnit = 30; // + robotLength;
    int squareDiagonal = 85;
    double halfSquareDiagonal = 36;
    int segmentTwoNorthDistance = 66;
    int finalSouth = 50;

    //int halvedSquareDiagonal = 42;
    

    int firstSegment = 45;
    int secondSegment = 54;
    // Inches north of center of the start zone that the robot starts
    int northOffset = 10;

    double powerMagnitude = .25;

    double secondOneUnit = 10;
    

    return new SequentialCommandGroup(
        // Assumes the robot starts at a 45 degree angle facing northeast
        new DriveTrainDriveInchesCommand(firstSegment, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(oneUnit, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainDriveInchesCommand(secondSegment, powerMagnitude, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(halfSquareDiagonal, powerMagnitude, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, 90),
        new DriveTrainDriveInchesCommand(halfSquareDiagonal, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(segmentTwoNorthDistance, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainDriveInchesCommand(segmentTwoNorthDistance + 8, powerMagnitude, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(halfSquareDiagonal, powerMagnitude, Calibrations.DRIVING_BACKWARD),
        // new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        //new DriveTrainDriveInchesCommand(secondOneUnit, powerMagnitude, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90),
        new DriveTrainDriveInchesCommand(halfSquareDiagonal, powerMagnitude, Calibrations.DRIVING_BACKWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -45),
        new DriveTrainDriveInchesCommand(segmentTwoNorthDistance + 65, powerMagnitude, Calibrations.DRIVING_BACKWARD),
        new DriveTrainDriveInchesCommand(finalSouth - 20, powerMagnitude, Calibrations.DRIVING_FORWARD),
        new DriveTrainTurnRelativeDegreesCommand(drivetrain, -90, .012),
        new DriveTrainDriveInchesCommand(finalSouth + 25, powerMagnitude, Calibrations.DRIVING_FORWARD)

        //new ShooterAutonomousShootCommand(),
        // new DriveTrainDriveInchesCommand(24, .25, Calibrations.DRIVING_FORWARD));
        //drive2FeetCommand
        );
  }
}

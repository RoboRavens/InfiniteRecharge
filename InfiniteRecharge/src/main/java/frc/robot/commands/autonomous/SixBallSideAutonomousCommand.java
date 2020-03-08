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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.DriveTrainDriveInchesCommand;
import frc.robot.commands.intake.IntakeExtendAndCollectCommand;
import frc.robot.commands.intake.IntakeRetractCommand;
import frc.robot.commands.shooter.ShooterAutonomousShootCommand;

/**
 * Turns the robot -> shoots -> turns back -> goes straight and picks up balls in trench
 */
public class SixBallSideAutonomousCommand {
    public static Command GenerateCommand() {
        var getBallsFromTrenchTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(4, 0, new Rotation2d(0)),
            Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig()
            );
        
            var returnToStartReversedTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(0, 0, new Rotation2d(0)),
            Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig().setReversed(true)
            );
        
            var getBallsFromTrenchCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(getBallsFromTrenchTrajectory);
            var returnToStartReversedTrajectoryCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(returnToStartReversedTrajectory);
        
            return new SequentialCommandGroup(
            //new DriveTrainTurnRelativeDegreesCommand(Robot.DRIVE_TRAIN_SUBSYSTEM, -25.5),
            new ShooterAutonomousShootCommand(),
            //new DriveTrainTurnRelativeDegreesCommand(Robot.DRIVE_TRAIN_SUBSYSTEM, 25.5),
            new ParallelDeadlineGroup(
                new DriveTrainDriveInchesCommand(200, .3, Calibrations.DRIVING_FORWARD),
                new IntakeExtendAndCollectCommand()
            ),
            new IntakeRetractCommand(),
            new DriveTrainDriveInchesCommand(200, .3, Calibrations.DRIVING_BACKWARD),
            //new DriveTrainTurnRelativeDegreesCommand(Robot.DRIVE_TRAIN_SUBSYSTEM, -25.5),
            new ShooterAutonomousShootCommand()
            //new DriveTrainTurnRelativeDegreesCommand(Robot.DRIVE_TRAIN_SUBSYSTEM, 25.5)
        );
    }
}

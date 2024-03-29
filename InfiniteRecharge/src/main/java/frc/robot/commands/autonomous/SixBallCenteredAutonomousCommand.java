/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.intake.IntakeCollectCommand;
import frc.robot.commands.intake.IntakeExtendCommand;
import frc.robot.commands.shooter.ShooterAutonomousShootCommand;
import frc.robot.commands.utility.SleepCommand;

public class SixBallCenteredAutonomousCommand {

  public static Command GenerateCommand() {
    var poseA = new Pose2d(3.558, -2.404, new Rotation2d(0));
    var poseB = new Translation2d(4.219, -1);
    var poseC = new Pose2d(5.2, -0.805, new Rotation2d(0));
    var poseD = new Pose2d(7.588, -0.805, new Rotation2d(0));
    var poseE = new Translation2d(4.558, -2);
    var poseAtoC = TrajectoryGenerator.generateTrajectory(poseA, List.of(poseB), poseC,
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig());

    var poseCtoD = TrajectoryGenerator.generateTrajectory(poseC, List.of(), poseD,
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig());

    var poseDtoAreverse = TrajectoryGenerator.generateTrajectory(poseD, List.of(poseE), poseA,
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig().setReversed(true));

    var poseAtoCcommand = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(poseAtoC);
    var poseCtoDcommand = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(poseCtoD);
    var poseDtoAreverseCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(poseDtoAreverse);

    return new SequentialCommandGroup(
        new InstantCommand(() -> Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.setOdometry(poseAtoC.getInitialPose()), Robot.DRIVE_TRAIN_SUBSYSTEM),
        new ShooterAutonomousShootCommand(),
        poseAtoCcommand,
        new IntakeExtendCommand(),
        new SleepCommand("wait for intake", 0.1),
        new ParallelDeadlineGroup(poseCtoDcommand, new IntakeCollectCommand()),
        poseDtoAreverseCommand,
        new ShooterAutonomousShootCommand(),
        new InstantCommand(() -> System.out.println("Drive Command Finished!")));
  }
}

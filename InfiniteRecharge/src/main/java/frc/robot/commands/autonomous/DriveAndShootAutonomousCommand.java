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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.DriveTrainDriveInchesCommand;
import frc.robot.commands.shooter.ShooterAutonomousShootCommand;
import frc.robot.commands.utility.SleepCommand;

/**
 * Add your docs here.
 */
public class DriveAndShootAutonomousCommand {
    public static Command GenerateCommand() {
        var drive1FootTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(1, 0, new Rotation2d(0)),
            Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig()
          );
      
          var drive1FootCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(drive1FootTrajectory);
      
          return new SequentialCommandGroup(
            new ShooterAutonomousShootCommand(),
            new DriveTrainDriveInchesCommand(24, .25, Calibrations.DRIVING_FORWARD)
          );
    }
}

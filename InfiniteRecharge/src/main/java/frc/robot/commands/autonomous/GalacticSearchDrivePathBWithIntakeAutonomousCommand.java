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

public class GalacticSearchDrivePathBWithIntakeAutonomousCommand {
  public static Command GenerateCommand() {
    
    return new ParallelCommandGroup(
        new IntakeExtendAndCollectCommand(),
        new GalacticSearchDrivePathBAutonomousCommand()
    );
  }
}

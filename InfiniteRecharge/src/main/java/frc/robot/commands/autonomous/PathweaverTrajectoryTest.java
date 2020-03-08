/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class PathweaverTrajectoryTest {
  public static Trajectory GetPathweaverTrajectoryForAuto() {
    try {
      var trajectory = TrajectoryUtil
          .fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/" + "Line" + ".wpilib.json"));
      return Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.reverseTrajectory(trajectory);
    } catch (IOException e) {
      e.printStackTrace();
    }

    return null;
  }

  public static Command GetReversePathweaverTrajectoryTest() {
    try {
      var trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/Line.wpilib.json"));
      var trajectoryToReverse = TrajectoryUtil
          .fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/ReverseLine.wpilib.json"));
      var reverseTrajectory = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.reverseTrajectory2(trajectoryToReverse);
      var trajectoryCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(trajectory);
      var reverseTrajectoryCommand = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(reverseTrajectory);
      return new SequentialCommandGroup(trajectoryCommand, reverseTrajectoryCommand);
    } catch (IOException e) {
      e.printStackTrace();
    }

    return null;
  }
}

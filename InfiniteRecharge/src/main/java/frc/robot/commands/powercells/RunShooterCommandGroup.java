/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.powercells;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.conveyance.ConveyanceShootWhileHeldCommand;
import frc.robot.commands.hopper.HopperFeedFullCommand;

public class RunShooterCommandGroup extends ParallelCommandGroup {
  public RunShooterCommandGroup() {
    super(new HopperFeedFullCommand(), new ConveyanceShootWhileHeldCommand());
  }
}

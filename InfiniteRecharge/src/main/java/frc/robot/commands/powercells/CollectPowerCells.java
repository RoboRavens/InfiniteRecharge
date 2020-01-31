/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.powercells;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.HarvestCommand;
import frc.robot.commands.intake.IntakeExtendCommand;

public class CollectPowerCells extends ParallelCommandGroup {
 
  public CollectPowerCells() {
    super(new IntakeExtendCommand(), new HarvestCommand());
  }
}

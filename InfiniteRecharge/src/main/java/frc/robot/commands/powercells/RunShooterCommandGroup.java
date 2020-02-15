/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.powercells;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.PrepareShotCommandGroup;

public class RunShooterCommandGroup extends SequentialCommandGroup {
  public RunShooterCommandGroup() {
    super(new PrepareShotCommandGroup(), new RunPowerCellsShooterCommandGroup());
  }
}

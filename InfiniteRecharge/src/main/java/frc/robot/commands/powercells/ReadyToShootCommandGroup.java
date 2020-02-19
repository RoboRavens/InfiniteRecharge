/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.powercells;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.conveyance.ConveyanceReadyToShootCommand;
import frc.robot.commands.hopper.HopperReadyShotCommand;

public class ReadyToShootCommandGroup extends ParallelCommandGroup {
  
  public ReadyToShootCommandGroup() {
    super(new ConveyanceReadyToShootCommand(), new HopperReadyShotCommand());
    System.out.println("READY_TO_SHOOT!!!");
  }
}

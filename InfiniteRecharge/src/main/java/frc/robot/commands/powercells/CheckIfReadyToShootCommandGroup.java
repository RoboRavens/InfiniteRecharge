/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.powercells;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.CheckIfTargetAlignedCommand;
import frc.robot.commands.shooter.CheckIfShooterAtRPMCommand;

public class CheckIfReadyToShootCommandGroup extends SequentialCommandGroup {

  // not finished
  public CheckIfReadyToShootCommandGroup() {
    super(new CheckIfTargetAlignedCommand(), 
          new CheckIfShooterAtRPMCommand(),
          // new RunCommand(() -> System.out.println("LEFT BUMPER PRESSED")).withInterrupt(() -> Robot.DRIVE_CONTROLLER.getButtonValue(ButtonCode.LEFTBUMPER)),
          new RunCommand(() -> System.out.println("LEFT TRIGGER PRESSED")).withInterrupt(() -> Robot.DRIVE_CONTROLLER.getAxisIsPressed(AxisCode.LEFTTRIGGER)));
  }
}

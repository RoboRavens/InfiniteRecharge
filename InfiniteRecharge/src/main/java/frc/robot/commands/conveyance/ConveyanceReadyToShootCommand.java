/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.conveyance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ConveyanceReadyToShootCommand extends CommandBase {
  public ConveyanceReadyToShootCommand() {
    addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ConveyanceReadyToShootCommand initialized");;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceSensor() == true) {
      System.out.println("Triggered by ball");
      Robot.CONVEYANCE_SUBSYSTEM.stopConveyance();
      Robot.HOPPER_SUBSYSTEM.setHopperMotors(0, 0);
    } else {
      Robot.CONVEYANCE_SUBSYSTEM.pistonBlock();
      Robot.CONVEYANCE_SUBSYSTEM.setConveyanceMotor(.5); //This may change
      Robot.HOPPER_SUBSYSTEM.setHopperMotors(1, 1); //Not sure what speed we want to run motors at
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

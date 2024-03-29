/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class HopperReadyShotCommand extends CommandBase {

  public HopperReadyShotCommand() {
    addRequirements(Robot.HOPPER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("HopperReadyShotCommand initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceSensor() == true) {
      Robot.HOPPER_SUBSYSTEM.stopHopperMotor();
    } else {
      Robot.HOPPER_SUBSYSTEM.fullForward(); // .feedForward()
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.HOPPER_SUBSYSTEM.stopHopperMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = false;
    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceSensor() == true) {
      isFinished = true;
    }
    return isFinished;
  }
}

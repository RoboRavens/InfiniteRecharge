/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CompressorTurnOffWhileShootingCommand extends CommandBase {
  /**
   * Creates a new CompressorTurnOffWhileShootingCommand.
   */
  public CompressorTurnOffWhileShootingCommand() {
    addRequirements(Robot.COMPRESSOR_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("CompressorTurnOffWhileShootingCommand init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.COMPRESSOR_SUBSYSTEM.setIsShooting(true);
    Robot.COMPRESSOR_SUBSYSTEM.stop();
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

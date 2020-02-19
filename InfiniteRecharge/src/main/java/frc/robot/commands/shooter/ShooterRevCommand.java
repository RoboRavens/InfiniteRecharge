/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterRevCommand extends CommandBase {

  private double _setRPM;
  public ShooterRevCommand(double setRPM) {
    this._setRPM = setRPM;
    addRequirements(Robot.SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShooterRevCommand Initialized!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.SHOOTER_SUBSYSTEM.setRPM(this._setRPM);
    System.out.println("REVING_SHOOTER!!!");
  }

  public boolean isFinished() {
  /*  boolean isFinished = false;
    if (Robot.SHOOTER_SUBSYSTEM.getIsAtRpmRange(Robot.SHOOTER_SUBSYSTEM.getTargetRPM()) == true) {
      isFinished = true;
    }
    return isFinished;
  }*/
  return false;
 }
}

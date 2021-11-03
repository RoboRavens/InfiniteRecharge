/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.conveyance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Calibrations;

public class ConveyanceReadyToShootCommand extends CommandBase {
  private final Timer _safetyTimer = new Timer();

  public ConveyanceReadyToShootCommand() {
    addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _safetyTimer.reset();
    _safetyTimer.start();
    //System.out.println("ConveyanceReadyToShootCommand initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("NOT TRIGGERED BY BALL YET");
    // Robot.CONVEYANCE_SUBSYSTEM.pistonBlock();
//    Robot.CONVEYANCE_SUBSYSTEM.wheelStop();
    Robot.CONVEYANCE_SUBSYSTEM.feederWheelForward();
    Robot.CONVEYANCE_SUBSYSTEM.setBeltNormalSpeedForward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.CONVEYANCE_SUBSYSTEM.stopBelt();
    _safetyTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = false;

    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceSensor() == true) {
      isFinished = true;
    }

    if (_safetyTimer.get() >= Calibrations.CONVEYANCE_SAFETY_TIMER_TIMEOUT) {
      isFinished = true;
    }

    return isFinished;
  }
}
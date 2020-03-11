/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterRevCommand extends CommandBase {

  private Timer _timer = new Timer();
  private double _timeTakenToRev = 0;

  public ShooterRevCommand() {
    addRequirements(Robot.SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShooterRevCommand Initialized!!");
    _timer.reset();
    _timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.SHOOTER_SUBSYSTEM.rev();
    if (Robot.SHOOTER_SUBSYSTEM.getIsInRpmRange()) {
      _timeTakenToRev = _timer.get();
      _timer.stop();
    }
    SmartDashboard.putNumber("secondsToRev", _timeTakenToRev);
    // System.out.println("REVVING_SHOOTER!!!");
  }

  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.SHOOTER_SUBSYSTEM.rpmManager.resetBallCount();
  }
}

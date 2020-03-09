/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.utility;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * SleepCommand is a useful dummy (genius) command for testing command groups.
 */
public class SleepCommand extends CommandBase {
  String _name;
  double _durationInSeconds;
  Timer _timer;

  /**
   * Given a name this command will announce it has started and then after the
   * duration it will announce it has been stopped. It will also display if it was
   * interrupted.
   * 
   * @param name
   * @param durationInSeconds
   */
  public SleepCommand(String name, double durationInSeconds) {
    _name = name;
    _durationInSeconds = durationInSeconds;
    _timer = new Timer();
  }

  @Override
  public void initialize() {
    System.out.println("command " + _name + " starting");
    _timer.start();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("command " + _name + " ended" + (interrupted ? " *interrupted*" : ""));
  }

  @Override
  public boolean isFinished() {
    if (_timer.get() > _durationInSeconds) {
      return true;
    }

    return false;
  }
}

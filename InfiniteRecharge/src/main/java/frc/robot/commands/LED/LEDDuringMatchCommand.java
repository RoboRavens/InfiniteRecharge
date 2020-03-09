/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import java.awt.Color;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;

public class LEDDuringMatchCommand extends CommandBase {
  private long _matchSecond;
  private boolean _colorSelected;

  private class BlinkCommand {
    private LEDBlinkCommand _blinkCommand;
    private boolean _hasRan;

    BlinkCommand(double color) {
      _blinkCommand = new LEDBlinkCommand(color, 2);
    }

    public void runCommand() {
      if (_hasRan == false) {
        CommandScheduler.getInstance().schedule(_blinkCommand);
        _hasRan = true;
      }
    }
  }

  private BlinkCommand After90SecondsCommand = new BlinkCommand(0.5);
  private BlinkCommand After60SecondsCommand = new BlinkCommand(0.3);
  private BlinkCommand After30SecondsCommand = new BlinkCommand(0.2);
  private BlinkCommand After10SecondsCommand = new BlinkCommand(0.1);

  public LEDDuringMatchCommand() {
    //addRequirements(Robot.PROGRAMMABLE_LED_SUBSYSTEM);
  }

  // Called just before this Command runs the first time
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run

  public void execute() {
    //_matchSecond = Robot.PROGRAMMABLE_LED_SUBSYSTEM.getMatchSecond();
    System.out.println(_matchSecond);
    _colorSelected = false;

    //if (Robot.PROGRAMMABLE_LED_SUBSYSTEM.isTeleopMode() == false) {
    //  return;
    //}

    this.setColorWhenAfter(10, After10SecondsCommand);
    this.setColorWhenAfter(30, After30SecondsCommand);
    this.setColorWhenAfter(60, After60SecondsCommand);
    this.setColorWhenAfter(90, After90SecondsCommand);
  }

  private void setColorWhenAfter(int second, BlinkCommand blinkCommand) {
    if (_colorSelected) {
      return;
    }

    if (_matchSecond >= second) {
      _colorSelected = true;
      blinkCommand.runCommand();
    }
  }

  // Make this return true when this Command no longer needs to run execute()

  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true

  public void end() {
  }

  // Called when another command which addRequirements one or more of the same
  // subsystems is scheduled to run

  public void interrupted() {
  }
}

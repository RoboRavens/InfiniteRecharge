/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.conveyance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ConveyanceReverseForDurationCommand extends CommandBase {
  double _durationInSeconds;
  Timer _timer;

  public ConveyanceReverseForDurationCommand(double durationInSeconds) {
    addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
    _durationInSeconds = durationInSeconds;
    _timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ConveyanceReverseForDurationCommand initialized");
    _timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.CONVEYANCE_SUBSYSTEM.setBeltMaxReverse();
    Robot.CONVEYANCE_SUBSYSTEM.feederWheelReverse();
    System.out.println("REVERSING_CONVEYOR!!!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (_timer.get() > _durationInSeconds) {
      return true;
    }

    return false;
  }
}

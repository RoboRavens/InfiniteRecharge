/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterTuneCommand extends CommandBase {

  public ShooterTuneCommand() {
    addRequirements(Robot.SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShooterTuneCommand Initialized!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("SHOOTER TUNE COMMAND HAS BEEN CALLED");
    // Robot.SHOOTER_SUBSYSTEM.setVelocityBySlider();
    System.out.println("The current RPM is: " + Robot.SHOOTER_SUBSYSTEM.getRPM());
  }

  public boolean isFinished() {
    return true;
  }
}
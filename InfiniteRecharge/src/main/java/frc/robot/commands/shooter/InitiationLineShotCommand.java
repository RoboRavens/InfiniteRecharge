/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterSubsystem;


public class InitiationLineShotCommand extends CommandBase {

  public InitiationLineShotCommand() {
    addRequirements(Robot.SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("InitiationLineShot Command Initialized!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("INITIATION LINE SHOT running at: " + Robot.SHOOTER_SUBSYSTEM.getVelocity());
    Robot.SHOOTER_SUBSYSTEM.setVelocityRaw(Calibrations.INIT_LINE_VELOCITY);
  }

  public boolean isFinished() {
    return true;
  }
}
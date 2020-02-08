/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class InitiationLineShotCommand extends CommandBase {
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterSubsystem shooter = new ShooterSubsystem();
  public InitiationLineShotCommand(ShooterSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    
    }
  // Called when the command is initially scheduled.
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("InitiationLineShot Command Initialized!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("INITIATION LINE SHOT COMMAND HAS BEEN CALLED");
    shooter.setRPM(1000);
  }

    public boolean isFinished() {
      return true;
    }
  }
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterAutonomousShootCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RunShooterAutonomousCommand extends SequentialCommandGroup {
  public RunShooterAutonomousCommand(double shooterRPM, double shooterDuration) {
    // super(new ShooterRevCommand(shooterRPM), new RunShooterForDurationCommandGroup(shooterDuration), new RevDownCommandGroup());
    super(new ShooterAutonomousShootCommand());
  }
}

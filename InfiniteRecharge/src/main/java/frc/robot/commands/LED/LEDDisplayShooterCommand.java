/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class LEDDisplayShooterCommand extends CommandBase {
  /**
   * Creates a new LEDDisplayShooter.
   */
  public LEDDisplayShooterCommand() {
    addRequirements(Robot.PROGRAMMABLE_LED_SUBSYSTEM, Robot.SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shooter LED display online!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.SHOOTER_SUBSYSTEM.getRPM() < 3) {
      //If shooter is off, turn off LED. Comparing to 3 to account for glitchy encoder signals
      Robot.PROGRAMMABLE_LED_SUBSYSTEM.setOff(); 

    } else if (Robot.SHOOTER_SUBSYSTEM.getIsInRpmRange()) {
      //If shooter is ready to shoot, turn green.
      Robot.PROGRAMMABLE_LED_SUBSYSTEM.setGreen(); 

    } else if (Robot.SHOOTER_SUBSYSTEM.getIsWideRpmRange()) {
      //If shooter is in the "almost range" (Calibrations.YELLOW_RPM_OFFSET)
      Robot.PROGRAMMABLE_LED_SUBSYSTEM.setYellow(); 

    } else {
      //If shooter isn't close, on target, or off, it has to be far from target, so show red.
      Robot.PROGRAMMABLE_LED_SUBSYSTEM.setRed();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

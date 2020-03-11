/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.ravenhardware.RavenBlinkin;
import frc.ravenhardware.RavenBlinkinPatternCodes;

public class CheckFlashCommand extends CommandBase {
  public RavenBlinkinPatternCodes CurrentCode;
  private Boolean flashOver;
  private static RavenBlinkin preparedBlinkin;
  /**
   * Creates a new CheckFlashCommand.
   */
  public CheckFlashCommand(RavenBlinkinPatternCodes PatternToSet, RavenBlinkin setBlinkin) {
    // Use addRequirements() here to declare subsystem dependencies.
    CurrentCode = PatternToSet;
    preparedBlinkin = setBlinkin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flashOver = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!preparedBlinkin.isFlashOver()) {
      switch (CurrentCode) {
        case FLASH_BLUE:
          preparedBlinkin.blinkBlue();
          break;

        case FLASH_GREEN:
          preparedBlinkin.blinkGreen();
          break;
          
        case FLASH_RED:
          preparedBlinkin.blinkRed();
          break;
          
        case FLASH_WHITE:
          preparedBlinkin.blinkWhite();
          break;

        case FLASH_YELLOW:
          preparedBlinkin.blinkYellow();
          break;
          
        default:
          preparedBlinkin.solidOff();
      }
    } else {
      flashOver = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    preparedBlinkin.solidOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flashOver;
  }
}

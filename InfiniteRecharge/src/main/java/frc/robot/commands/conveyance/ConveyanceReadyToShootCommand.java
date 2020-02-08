/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.conveyance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

public class ConveyanceReadyToShootCommand extends CommandBase {
  private boolean isFinished = false;
  private final Timer m_timer = new Timer();
  public ConveyanceReadyToShootCommand() {
    addRequirements(Robot.CONVEYANCE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    System.out.println("ConveyanceReadyToShootCommand initialized");
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceSensor() == true) {
      System.out.println("TRIGGERD BY BALL");
      Robot.CONVEYANCE_SUBSYSTEM.stopConveyance();
    } else {
      System.out.println("NOT TRIGGERED BY BALL YET");
      Robot.CONVEYANCE_SUBSYSTEM.pistonBlock();
      Robot.CONVEYANCE_SUBSYSTEM.setNormalSpeedConveyance();
    }
    
    if (Robot.CONVEYANCE_SUBSYSTEM.getConveyanceSensor() == false) {
        m_timer.start();
        isFinished = false;
      }
    
    if (m_timer.get() >= 10) {
      isFinished = true;
      }

    }    

    


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}


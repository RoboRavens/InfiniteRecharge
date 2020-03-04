/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.controls.ButtonCode;
import frc.robot.Calibrations;

public class ShooterRevCommand extends CommandBase {

  public ShooterRevCommand() {
    addRequirements(Robot.SHOOTER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShooterRevCommand Initialized!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.OPERATION_PANEL_2.getButtonValue(ButtonCode.SETSHOTCLOSECONTROLPANEL)) {
      Robot.SHOOTER_SUBSYSTEM.setRPM(Calibrations.CLOSE_TRENCH_SHOT_RPM);
      Robot.SHOOTER_SUBSYSTEM.setPidCloseTrench();
    } else if (Robot.OPERATION_PANEL_2.getButtonValue(ButtonCode.SETSHOTFARCONTROLPANEL)) {
      Robot.SHOOTER_SUBSYSTEM.setRPM(Calibrations.FAR_TRENCH_SHOT_RPM);
      Robot.SHOOTER_SUBSYSTEM.setPidFarTrench();
    } else {
      Robot.SHOOTER_SUBSYSTEM.setRPM(Calibrations.INIT_LINE_RPM);
      Robot.SHOOTER_SUBSYSTEM.setPidInit();
    }

    System.out.println("REVING_SHOOTER!!!");
  }

  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Robot.CONVEYANCE_SUBSYSTEM.stopBelt();
    // Robot.CONVEYANCE_SUBSYSTEM.wheelStop();
    // Robot.HOPPER_SUBSYSTEM.stopHopperMotor();
  }
}

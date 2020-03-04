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

  private double _setRPM;
  public ShooterRevCommand(double setRPM) {
    this._setRPM = setRPM;
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
    if (Robot.OPERATION_PANEL_2.getButtonValue(ButtonCode.SETSHOTCONTROLPANEL)){
      Robot.SHOOTER_SUBSYSTEM.setRPM(Calibrations.TRENCH_SHOT);
      //_shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, Calibrations.SHOOTER_KF_FURTHER, TalonSRXConstants.kTimeoutMs);
      //System.out.println("Setting!");
    }
    else {
      Robot.SHOOTER_SUBSYSTEM.setRPM(Calibrations.INITIATION_LINE_SHOT);
      //_shooterMotor.config_kF(TalonSRXConstants.kPIDLoopIdx, Calibrations.SHOOTER_KF, TalonSRXConstants.kTimeoutMs);
    }
    
    System.out.println("REVING_SHOOTER!!!");

    // if (Robot.SHOOTER_SUBSYSTEM.getIsAtRpmRange(Calibrations.TARGET_RANGE)){
      // isFinished = true;
    // } 

    /*
boolean isInRpmRange = true;
boolean isAtAngle = true;

    if (isInRpmRange && isAtAngle) {
      Robot.CONVEYANCE_SUBSYSTEM.setBeltMaxForward();
      Robot.CONVEYANCE_SUBSYSTEM.feederWheelForward();
      Robot.HOPPER_SUBSYSTEM.feedFullSpeed();
    }
    else {
      Robot.CONVEYANCE_SUBSYSTEM.stopBelt();
      Robot.CONVEYANCE_SUBSYSTEM.wheelStop();
      Robot.HOPPER_SUBSYSTEM.stopHopperMotor();
    }
    */
  }

  public boolean isFinished() {
  /*  boolean isFinished = false;
    if (Robot.SHOOTER_SUBSYSTEM.getIsAtRpmRange(Robot.SHOOTER_SUBSYSTEM.getTargetRPM()) == true) {
      isFinished = true;
    }
    return isFinished;
  }*/
  return false;
 }

  // Called once the command ends or is interrupted.
  @Override
    public void end(boolean interrupted) {
      //Robot.CONVEYANCE_SUBSYSTEM.stopBelt();
      //Robot.CONVEYANCE_SUBSYSTEM.wheelStop();
      //Robot.HOPPER_SUBSYSTEM.stopHopperMotor();
  }
}

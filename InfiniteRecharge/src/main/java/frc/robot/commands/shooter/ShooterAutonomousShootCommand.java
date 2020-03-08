/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Calibrations;
import frc.robot.Robot;

public class ShooterAutonomousShootCommand extends CommandBase {
  private Timer _timer = new Timer();
  
  public ShooterAutonomousShootCommand() {
    addRequirements(Robot.SHOOTER_SUBSYSTEM, Robot.CONVEYANCE_SUBSYSTEM, Robot.HOPPER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.reset();
    _timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.SHOOTER_SUBSYSTEM.rev();

    System.out.print("Angle: " + Robot.LIMELIGHT_SUBSYSTEM.isAlignedToTarget());
    System.out.println(" RPM: " + Robot.SHOOTER_SUBSYSTEM.getRPM());
    System.out.println(" RTS: " + Robot.SHOOTER_SUBSYSTEM.readyToShoot());
    System.out.println();

    if (Robot.SHOOTER_SUBSYSTEM.readyToShootAuto()) {
      Robot.CONVEYANCE_SUBSYSTEM.feederWheelForward();
      Robot.CONVEYANCE_SUBSYSTEM.setBeltMaxForward();
      Robot.HOPPER_SUBSYSTEM.fullForward();
    }
    else {
      Robot.CONVEYANCE_SUBSYSTEM.stopBelt();
      Robot.CONVEYANCE_SUBSYSTEM.wheelStop();
      Robot.HOPPER_SUBSYSTEM.stopHopperMotor();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.CONVEYANCE_SUBSYSTEM.stopBelt();
    Robot.CONVEYANCE_SUBSYSTEM.wheelStop();
    Robot.HOPPER_SUBSYSTEM.stopHopperMotor();
    Robot.SHOOTER_SUBSYSTEM.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.SHOOTER_SUBSYSTEM.getBallsShot() == 3 || _timer.get() > Calibrations.AUTONOMOUS_SHOOTER_3_BALL_TIMEOUT) {
      return true;
    }
    return false;
  }
}

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
  Timer timer = new Timer();
  
  /**
   * 
   * Creates a new ShooterAutonomousShootCommand.
   */
  public ShooterAutonomousShootCommand() {
    
    addRequirements(Robot.SHOOTER_SUBSYSTEM, Robot.CONVEYANCE_SUBSYSTEM, Robot.HOPPER_SUBSYSTEM);

    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot.SHOOTER_SUBSYSTEM.setRPM(Calibrations.INIT_LINE_RPM);
    Robot.SHOOTER_SUBSYSTEM.setRPM(120);
    // System.out.println("REVING_SHOOTER!!!");

    System.out.print("Angle: " + Robot.LIMELIGHT_SUBSYSTEM.isAlignedToTarget());
    System.out.print(" RPM: " + Robot.SHOOTER_SUBSYSTEM.getIsInInitiationLineRpmRange());
    System.out.println(" RPM: " + Robot.SHOOTER_SUBSYSTEM.getRPM());
    System.out.println(" RTS: " + Robot.SHOOTER_SUBSYSTEM.readyToShoot());
    System.out.println();

    if ((Robot.SHOOTER_SUBSYSTEM.readyToShootAuto() && timer.get() > 3) || (timer.get() > 6 && timer.get() < 9)) {
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
    if (this.timer.get() >= 9) {
      return true;
    }
    return false;
  }
}

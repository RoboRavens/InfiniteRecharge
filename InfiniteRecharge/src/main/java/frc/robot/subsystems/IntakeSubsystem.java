/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX _intakeMotor;
  private Timer _safetyTimer = new Timer();

  public IntakeSubsystem() {

  }

  public void initialize() {

  }

  public void periodic() {

  }
}

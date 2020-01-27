/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HopperSubsystem extends SubsystemBase {

  private TalonSRX _hopperMotor;
  private TalonSRX _hopperMotor2;
  
  public HopperSubsystem() {
    _hopperMotor = new TalonSRX(RobotMap.hopperMotor);
    _hopperMotor2 = new TalonSRX(RobotMap.hopperMotor2);
  }

  public void initialize() {

  }

  public void periodic() {
    
  }
}

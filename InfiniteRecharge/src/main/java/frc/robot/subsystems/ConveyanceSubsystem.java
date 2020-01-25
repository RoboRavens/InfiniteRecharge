/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.RobotMap;

public class ConveyanceSubsystem extends SubsystemBase {

  private TalonSRX _conveyanceMotor;
  private TalonSRX _conveyanceMotor2;
  private BufferedDigitalInput _conveyanceSensor;

  public ConveyanceSubsystem() {
    _conveyanceMotor = new TalonSRX(RobotMap.conveyanceMotor);
    _conveyanceMotor2 = new TalonSRX(RobotMap.conveyanceMotor2);
    _conveyanceSensor = new BufferedDigitalInput(RobotMap.conveyanceSensor);
  }

  public void initialize() {

  }

  public void periodic() {

  }
}

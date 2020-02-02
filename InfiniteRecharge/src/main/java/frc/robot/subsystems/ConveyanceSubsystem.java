/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import frc.robot.RobotMap;
import frc.robot.commands.conveyance.ConveyanceStopCommand;

public class ConveyanceSubsystem extends SubsystemBase {

  private TalonSRX _conveyanceMotor; // 1 talon SRX will run two bag motors on robot
  private BufferedDigitalInput _conveyanceSensor;

  private Solenoid _pistonBlock;
  private Solenoid _pistonUnblock;

  public ConveyanceSubsystem() {
    this.initialize();
    _conveyanceMotor = new TalonSRX(RobotMap.conveyanceMotor);
    _conveyanceSensor = new BufferedDigitalInput(RobotMap.conveyanceSensor);

    _pistonBlock = new Solenoid(RobotMap.pistonBlockSolenoid);
    _pistonUnblock = new Solenoid(RobotMap.pistonUnblockSolenoid);
  }

  public void initialize() {
    setDefaultCommand(new ConveyanceStopCommand());
  }

  public void periodic() {

  }

  public void setReverseConveyance() {
    this.setConveyanceMotor(-1.0);
  }

  public void setMaxConveyance() {
    this.setConveyanceMotor(1.0);
  }

  public void stopConveyance() {
    this.setConveyanceMotor(0);
  }

  public void setConveyanceMotor(double magnitude) {
    _conveyanceMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public boolean getConveyanceSensor() {
    System.out.println("Got sensor");
    return _conveyanceSensor.get();
  }

  public void pistonBlock() {
    System.out.println("Blocking");
    _pistonUnblock.set(false);
    _pistonBlock.set(true);
  }

  public void pistonUnblock() {
    System.out.println("Unblocking");
    _pistonUnblock.set(true);
    _pistonBlock.set(false);
  }
}

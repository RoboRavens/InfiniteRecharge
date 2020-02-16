/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedDigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Calibrations;
import frc.robot.RobotMap;

public class ConveyanceSubsystem extends SubsystemBase {

  // 1 talon SRX will run two bag motors on robot
  private TalonSRX _conveyanceMotor;
  private BufferedDigitalInput _conveyanceSensor;

  private Solenoid _pistonBlock;
  private Solenoid _pistonUnblock;

  public ConveyanceSubsystem() {
    this.initialize();
    _conveyanceMotor = new TalonSRX(RobotMap.CONVEYANCE_MOTOR);
    _conveyanceSensor = new BufferedDigitalInput(RobotMap.CONVEYANCE_SENSOR);

    _pistonBlock = new Solenoid(RobotMap.PISTON_BLOCK_SOLENOID);
    _pistonUnblock = new Solenoid(RobotMap.PISTON_UNBLOCK_SOLENOID);
  }

  public void initialize() {
    // setDefaultCommand(new ConveyanceStopCommand());
  }

  public void periodic() {

  }

  public void setReverse() {
    this.runAtPower(Calibrations.CONVEYANCE_FULL_SPEED_REVERSE);
  }

  public void setMaxForward() {
    this.runAtPower(Calibrations.CONVEYANCE_FULL_SPEED);
  }

  public void stop() {
    this.runAtPower(Calibrations.CONVEYANCE_STOP);
  }

  public void setNormalSpeedForward() {
    this.runAtPower(Calibrations.CONVEYANCE_NORMAL_SPEED);
  }

  public void setNormalSpeedReverse() {
    this.runAtPower(Calibrations.CONVEYANCE_NORMAL_REVERSE_SPEED);
  }

  public void runAtPower(double magnitude) {
    _conveyanceMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public boolean getConveyanceSensor() {
    // Sysouts are for testing
    System.out.println("Got sensor");
    return _conveyanceSensor.get();
  }

  public void pistonBlock() {
    // Sysouts are for testing
    System.out.println("Blocking");
    _pistonUnblock.set(false);
    _pistonBlock.set(true);
  }

  public void pistonUnblock() {
    // Sysouts are for testing
    System.out.println("Unblocking");
    _pistonUnblock.set(true);
    _pistonBlock.set(false);
  }
}

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
import frc.ravenhardware.IRavenTalon;
import frc.ravenhardware.RavenTalonSRX;
//import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ConveyanceSubsystem extends SubsystemBase {

  // 1 talon SRX will run two bag motors on robot
  private TalonSRX _conveyanceMotor;
  private RavenTalonSRX _conveyanceWheel;
  private BufferedDigitalInput _conveyanceSensor;

  //private Solenoid _pistonBlock;
  //private Solenoid _pistonUnblock;

  public ConveyanceSubsystem() {
    this.initialize();
    _conveyanceMotor = new TalonSRX(RobotMap.CONVEYANCE_MOTOR);
    _conveyanceSensor = new BufferedDigitalInput(RobotMap.CONVEYANCE_SENSOR);
    _conveyanceWheel = new RavenTalonSRX(RobotMap.CONVEYANCE_WHEEL, 0, null, 0, false);

    this._conveyanceWheel.setCurrentLimit(Calibrations.CONVEYANCE_FEEDER_LIMIT);

    //_pistonBlock = new Solenoid(RobotMap.PISTON_BLOCK_SOLENOID);
    //_pistonUnblock = new Solenoid(RobotMap.PISTON_UNBLOCK_SOLENOID);
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

  public void runWheelAtPercentPower(double magnitude) {
    _conveyanceWheel.set(magnitude);
  }
  
  public void feederWheelForward() {
    this.runWheelAtPercentPower(Calibrations.CONVEYANCE_FEEDER_SPEED);
  }

  public void wheelStop() {
    this.runWheelAtPercentPower(Calibrations.CONVEYANCE_FEEDER_STOP);
  }

  public void feederWheelReverse() {
    this.runWheelAtPercentPower(Calibrations.CONVEYANCE_REVERSE_FEEDER);
  }

  public boolean getConveyanceSensor() {
    // Sysouts are for testing
    System.out.println("Got sensor");
    return _conveyanceSensor.get();
  }

  public void pistonBlock() {
    // Sysouts are for testing
    System.out.println("Blocking");
    //_pistonUnblock.set(false);
    //_pistonBlock.set(true);
  }

  public void pistonUnblock() {
    // Sysouts are for testing
    System.out.println("Unblocking");
    //_pistonUnblock.set(true);
    //_pistonBlock.set(false);
  }
}

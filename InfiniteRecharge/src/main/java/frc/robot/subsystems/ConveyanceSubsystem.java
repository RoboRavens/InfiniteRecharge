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
  private RavenTalonSRX _feederWheelMotor;
  //private TalonSRX _conveyanceWheel;
  private BufferedDigitalInput _conveyanceSensor;

  //private Solenoid _pistonBlock;
  //private Solenoid _pistonUnblock;

  public ConveyanceSubsystem() {
    _conveyanceMotor = new TalonSRX(RobotMap.CONVEYANCE_MOTOR);
    _conveyanceSensor = new BufferedDigitalInput(RobotMap.CONVEYANCE_SENSOR);
    _feederWheelMotor = new RavenTalonSRX(RobotMap.CONVEYANCE_WHEEL, "Conveyance Wheel", false);
    //_conveyanceWheel = new TalonSRX(RobotMap.CONVEYANCE_WHEEL);

    _feederWheelMotor.setMaxPower(Calibrations.CONVEYANCE_FEEDER_SPEED);
    _feederWheelMotor.setCurrentLimit(Calibrations.CONVEYANCE_FEEDER_LIMIT);
  }

  public void periodic() {

  }

  public void setBeltMaxReverse() {
    this.runBeltAtPercentPower(Calibrations.CONVEYANCE_FULL_SPEED_REVERSE);
  }

  public void setBeltMaxForward() {
    this.runBeltAtPercentPower(Calibrations.CONVEYANCE_FULL_SPEED);
  }

  public void stopBelt() {
    this.runBeltAtPercentPower(Calibrations.CONVEYANCE_STOP);
  }

  public void setBeltNormalSpeedForward() {
    this.runBeltAtPercentPower(Calibrations.CONVEYANCE_NORMAL_SPEED);
  }

  public void setBeltNormalSpeedReverse() {
    this.runBeltAtPercentPower(Calibrations.CONVEYANCE_NORMAL_REVERSE_SPEED);
  }

  private void runBeltAtPercentPower(double magnitude) {
    _conveyanceMotor.set(ControlMode.PercentOutput, magnitude);
  }

  private void runWheelAtPercentPower(double magnitude) {
    _feederWheelMotor.set(magnitude);
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

  public void defaultCommand() {
    //setDefaultCommand(new ConveyanceStopCommand());
    this.stopBelt();
    this.wheelStop();
  }
}

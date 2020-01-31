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
import frc.robot.RobotMap;

public class TowerSubsystem extends SubsystemBase {

  private TalonSRX _towerMotor; // 1 talon SRX will run two bag motors on robot
  private BufferedDigitalInput _towerSensor;

  public TowerSubsystem() {
    this.initialize();
    _towerMotor = new TalonSRX(RobotMap.towerMotor);
    _towerSensor = new BufferedDigitalInput(RobotMap.towerSensor);
  }

  public void initialize() {

  }

  public void periodic() {

  }

  public void stopPowerCells() {
    this.setTowerMotor(0);
  }

  public void setTowerMotor(double magnitude) {
    _towerMotor.set(ControlMode.PercentOutput, magnitude);
  }

  public boolean getTowerSensor() {
    return _towerSensor.get();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Calibrations;
import frc.robot.Robot;

public class ShooterRpmManager {
    LinkedList<Integer> rpms;
    private int _listSize;
    private boolean _readyToCountBalls = true;
    private int _ballCount = 0;

    private Timer _timer = new Timer();
    private double _lowestRPM = 0;
    private boolean _wasInRpmRangeLastCycle = false;
    private double _timeWhenNotInRangeDetected;

    public ShooterRpmManager(int listSize) {
        rpms = new LinkedList<Integer>();
        _listSize = listSize;
        _timer.start();
    }
    
    public int getBallCount() {
        return _ballCount;
    }

    public void resetBallCount() {
        _ballCount = 0;
    }

    public void updateRpms(int currentValue) {
        rpms.add(currentValue);

        if (rpms.size() > _listSize) {
			rpms.remove();
		}
    }

    private int calculatePeriodDifference() {
        var firstValue = rpms.get(0);
        var lastValue = rpms.get(_listSize - 1);

        return firstValue - lastValue;
    }
    
    public void updateBallsShot() {
        var diff = calculatePeriodDifference();

        if (_readyToCountBalls) {
            if (diff >= Calibrations.ACCEPTABLE_RPM_DROP_TO_COUNT_SHOT) {
                _ballCount++;
                _readyToCountBalls = false;
            }
        }
        else {
            if (diff <= Calibrations.ACCEPTABLE_RPM_DROP_TO_COUNT_SHOT) {
                _readyToCountBalls = true;
            }
        }
    }

    public void calculateSecondsToRevUpShot() {
        var rpm = Robot.SHOOTER_SUBSYSTEM.getRPM();
        if (Robot.SHOOTER_SUBSYSTEM.getIsInRpmRange() == false) {
          if (_wasInRpmRangeLastCycle) {
            System.out.println(_timer.get() + " RPM not in range for first time at " + rpm);
            _timeWhenNotInRangeDetected = _timer.get();
          }
    
          _lowestRPM = Math.min(rpm, _lowestRPM);
          _wasInRpmRangeLastCycle = false;
        } else {
          if (_wasInRpmRangeLastCycle == false) {
            // System.out.println(_timer.get() + " Above target for first time at " + rpm);
            System.out.println("Reached " + Robot.SHOOTER_SUBSYSTEM.getShot().name + " RPM of " + Robot.SHOOTER_SUBSYSTEM.getShot().targetRpm + " from " + _lowestRPM + " after "
                + (_timer.get() - _timeWhenNotInRangeDetected) + " seconds");
            _lowestRPM = Robot.SHOOTER_SUBSYSTEM.getRPM();
          }
    
          _wasInRpmRangeLastCycle = true;
        }
      }
}

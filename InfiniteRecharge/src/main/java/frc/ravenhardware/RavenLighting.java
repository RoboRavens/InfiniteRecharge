package frc.ravenhardware;

import frc.robot.Calibrations;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Timer;

public class RavenLighting {
	private Relay _mainArray;
	private Timer _timer;
	private boolean _toggling;
	private boolean _onForSeconds = false;
	private double _secondsDuration = 0;

	public RavenLighting(Relay relay) {
		_mainArray = relay;
		_timer = new Timer();
		_toggling = false;
	}

	public void turnOn() {
		cancelToggle();
		_mainArray.set(Value.kForward);
	}

	public void turnOff() {
		cancelToggle();
		_mainArray.set(Value.kOff);
	}

	public void quickToggle() {
		// If not already toggling, initialize a toggle sequence.
		// If already toggling, no need to do anything.
		if (_toggling == false) {
			_timer.reset();
			_timer.start();
			_toggling = true;
		}
	}

	public void turnOnForSeconds(double seconds) {
		this.turnOn();
		_timer.start();
		_onForSeconds = true;
		_secondsDuration = seconds;
	}

	public void maintainSecondsState() {
		if (_onForSeconds == false) {
			return;
		}

		if (_timer.get() < _secondsDuration) {
			this.turnOn();
		} else {
			this.turnOff();
			_onForSeconds = false;
		}
	}

	public void cancelToggle() {
		_toggling = false;
		_timer.stop();
		_timer.reset();
	}

	public void maintainState() {
		maintainSecondsState();

		// If not toggling, this method does nothing.
		if (_toggling == false) {
			return;
		}

		double timerMs = _timer.get() * 1000;

		if (timerMs > Calibrations.lightingFlashTotalDurationMs) {
			cancelToggle();
			return;
		}

		Value lightsValue;

		// If we haven't returned yet, we're toggling.
		// The duration of each on/off cycle is the total flash duration,
		// divided by the number of flashes. An individual on/off is half of that.
		double cycleDuration = Calibrations.lightingFlashTotalDurationMs / Calibrations.lightingFlashes;

		// Modding the timer by the cycle duration gives us the time elapsed in the
		// current cycle.
		double msElapsedInCurrentCycle = timerMs % cycleDuration;

		// The lights should be on for the first half of each cycle, and off for the
		// second half.
		if (msElapsedInCurrentCycle * 2 < cycleDuration) {
			lightsValue = Value.kOn;
		} else {
			lightsValue = Value.kOff;
		}

		_mainArray.set(lightsValue);
	}
}
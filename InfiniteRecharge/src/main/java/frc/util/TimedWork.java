package frc.util;

import edu.wpi.first.wpilibj.Timer;

public class TimedWork {
	private double _secondsBetweenWork;
	private Timer _timer;

	public TimedWork(double secondsBetweenWork) {
		_secondsBetweenWork = secondsBetweenWork;
	}

	public boolean getWorkAllowed() {
		// always allow work the first time
		if (_timer == null) {
			_timer = new Timer();
			_timer.start();
			return true;
		}

		// allow work after period has passed
		if (_timer.get() > _secondsBetweenWork) {
			_timer.reset();
			return true;
		}

		return false;
	}
}

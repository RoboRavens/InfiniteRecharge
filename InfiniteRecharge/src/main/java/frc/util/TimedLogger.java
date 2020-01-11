package frc.util;

import edu.wpi.first.wpilibj.Timer;

public class TimedLogger {
	private double _secondsBetweenPrint;
	private Timer _timer;

	public TimedLogger(double secondsBetweenPrint) {
		_secondsBetweenPrint = secondsBetweenPrint;
	}

	public void log(String message) {
		// always log the first time
		if (_timer == null) {
			_timer = new Timer();
			_timer.start();
			System.out.println(message);
			return;
		}

		// log after period has passed
		if (_timer.get() > _secondsBetweenPrint) {
			_timer.reset();
			System.out.println(message);
		}
	}
}

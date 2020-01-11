package frc.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;

public class LoggerOverlord {
	private Map<LoggerOverlordLogID, String> _map = new HashMap<LoggerOverlordLogID, String>();
	private double _timeBetweenPrints;
	private Timer _timer;

	public LoggerOverlord(double timeBetweenPrints) {
		_timeBetweenPrints = timeBetweenPrints;
		_timer = new Timer();
		_timer.start();
	}

	public void log(LoggerOverlordLogID uniqueID, String message) {
		if (_timer.get() > _timeBetweenPrints) {
			this.print();
			_timer.reset();
		}

		_map.put(uniqueID, message);
	}

	private void print() {
		System.out.println("-------------");
		System.out.println("time" + Timer.getFPGATimestamp());
		ArrayList<String> outputList = new ArrayList<String>();

		int maxNameLength = 0;
		for (Map.Entry<LoggerOverlordLogID, String> entry : _map.entrySet()) {
			int nameLength = entry.getKey().name().length();
			maxNameLength = Math.max(maxNameLength, nameLength);
		}

		for (Map.Entry<LoggerOverlordLogID, String> entry : _map.entrySet()) {
			outputList.add(padRight(entry.getKey().name(), maxNameLength) + ": " + entry.getValue());
		}

		Collections.sort(outputList);
		for (String logLine : outputList) {
			System.out.println(logLine);
		}

		System.out.println("-------------");

		for (Map.Entry<LoggerOverlordLogID, String> entry : _map.entrySet()) {
			_map.put(entry.getKey(), "N/A");
		}
	}

	private String padRight(String s, int n) {
		return String.format("%1$-" + n + "s", s);
	}
}

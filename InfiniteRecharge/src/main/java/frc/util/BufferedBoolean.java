package frc.util;

import java.util.LinkedList;

public class BufferedBoolean {
	protected int _bufferSize;
	
	LinkedList<Boolean> values;
	
	public BufferedBoolean() {
		values = new LinkedList<Boolean>();
		_bufferSize = 29;
	}
	
	public BufferedBoolean(int bufferSize) {
		_bufferSize = 29;
	}
	
	public BufferedBoolean(double seconds) {
		// The driver station operates at 50hz (fifty cycles per second.)
		// One second divided by 50 cycles = one cycle every 20 milliseconds.
		// Multiply seconds by fifty to get the buffer size.
		int bufferSize = (int) Math.round(seconds * 50);
		
		_bufferSize = bufferSize;
	}
	
	// Adds the current value to the list, and
	// removes the first item if the list is larger than the list size.
	public void maintainState(boolean currentValue) {
		values.add(currentValue);
		
		if (values.size() > _bufferSize) {
			values.remove();
		}
	}
	
	public boolean get() {
		int trues = 0;
		
		// Count the instances of "true" in the list values.
		for (Boolean value : values) {
			if (value) {
				trues++;
			}
		}
		
		boolean bufferedValue = (trues * 2 > _bufferSize);
		
		return bufferedValue;
	}

}
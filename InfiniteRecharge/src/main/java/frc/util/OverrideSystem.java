package frc.util;

public class OverrideSystem {
	private boolean _override1;

	public void setOverride1(boolean val) {
		_override1 = val;
	}

	public boolean getOverride1() {
		return _override1;
	}

	public boolean getIsAtLimit(boolean encoderLimit, boolean switchLimit) {
		if (switchLimit == true) {
			return true;
		}

		if (_override1 == true) {
			return false;
		}

		return encoderLimit;
	}
}

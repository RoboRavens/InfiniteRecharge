package frc.ravenhardware;

import frc.controls.AxisCode;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.RobotMap;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class RavenTank {

	private Timer _gyroCooldownTimer;

	AHRS orientationGyro = new AHRS(SPI.Port.kMXP);

	AHRS.BoardYawAxis boardYawAxis;
	private double _lastAccelerationX;
	private double _lastAccelerationY;
	private double _highestJerkX;
	private double _highestJerkY;
	private double _slewRate;

	protected int driveMode;
	protected int gyroMode;
	private boolean _cutPower;
	protected double gyroTargetHeading;

	protected boolean automatedDrivingEnabled = false;
	protected int automatedDrivingDirection = Calibrations.drivingForward;
	protected double automatedDrivingSpeed = 0;

	protected boolean hasHitObstacle = false;
	protected boolean drivingThroughObstacle = false;
	protected boolean turning = false;
	protected boolean waiting = false;

	public double gyroAdjust;
	double _gyroAdjustmentScaleFactor = Calibrations.GYRO_ADJUSTMENT_DEFAULT_SCALE_FACTOR;

	public boolean userControlOfCutPower = true;

	RavenTalon driveLeft = new RavenTalon(RobotMap.LEFT_DRIVE_CHANNEL_1, RobotMap.LEFT_DRIVE_CHANNEL_2, "MotorLeft", _slewRate);
	RavenTalon driveRight = new RavenTalon(RobotMap.RIGHT_DRIVE_CHANNEL_1, RobotMap.RIGHT_DRIVE_CHANNEL_2, "MotorRight", _slewRate);

	public RavenTank() {
		initializeRavenTank();
	}

	private void initializeRavenTank() {
		_slewRate = Calibrations.SLEW_RATE_MAXIMUM;

		_gyroCooldownTimer = new Timer();

		setDriveMode(Calibrations.defaultDriveMode);
		setCutPower(false);

		setGyroMode(Calibrations.defaultGyroMode);
		gyroTargetHeading = setGyroTargetHeadingToCurrentHeading();
	}

	public double deadband(double input) {
		double output = input;

		if (Math.abs(output) < Calibrations.deadbandMagnitude) {
			output = 0;
		}

		return output;
	}

	public void drive(double left, double rightY, double rightX) {
		left = deadband(left);
		rightY = deadband(rightY);
		rightX = deadband(rightX);
		
		fpsTank(left, rightX);
	}

	public void driveLeftSide(double magnitude) {
		driveLeft.set(magnitude);
	}

	public void driveRightSide(double magnitude) {
		driveRight.set(magnitude);
	}

	public void fpsTank(double translation, double turn) {
		double adjustedTurn = getFedForwardDriveValue(turn, Calibrations.TURN_FEED_FORWARD_MAGNITUDE, false);
		double squaredTranslation = getFedForwardDriveValue(translation, Calibrations.TRANSLATION_FEED_FORWARD_MAGNITUDE, false);
		// double squaredTranslation = Math.copySign(Math.pow(translation, 2), translation);

		if (Robot.DRIVE_CONTROLLER.getAxis(AxisCode.LEFTTRIGGER) > .25) {
			Robot.LIMELIGHT_SUBSYSTEM.turnLEDOn();
			fpsTankChooseLimelightOrManual(squaredTranslation, adjustedTurn);
		}
		else {
			Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
			fpsTankManual(squaredTranslation, adjustedTurn);
		}
	}

	public void fpsTankChooseLimelightOrManual (double translation, double turn) {
		if (Robot.LIMELIGHT_SUBSYSTEM.hasTarget()) {
			fpsTankLimelight(translation);
		} else {
			fpsTankManual(translation, turn);
		}
	}

	public void fpsTankLimelight(double translation) {
		// This method simply sets the gyro target, it doesn't actually turn the robot.
		// As a result, the gyro adjust value below will effectually turn the robot.
		Robot.LIMELIGHT_SUBSYSTEM.turnToTarget();
	
		if (_cutPower) {
			translation *= Calibrations.CUT_POWER_MODE_MOVEMENT_RATIO;
		}
	
		double gyroAdjust = getStaticGyroAdjustment();
		double leftFinal = translation * -1 - gyroAdjust;
		double rightFinal = translation - gyroAdjust;

		this.driveLeftSide(leftFinal);
		this.driveRightSide(rightFinal);
	}

	public void fpsTankManual(double translation, double turn) {
		if (_cutPower) {
			translation *= Calibrations.CUT_POWER_MODE_MOVEMENT_RATIO;
			turn *= Calibrations.CUT_POWER_MODE_TURN_RATIO;
		}

		// Apply a small reduction to turning magnitude based on the magnitude of
		// translation.
		turn = getScaledTurnFromTranslation(translation, turn);

		double gyroAdjust = getTurnableGyroAdjustment(turn);

		

		double leftFinal = (translation - turn) * -1 - gyroAdjust;
		double rightFinal = (translation + turn) - gyroAdjust;

		this.driveLeftSide(leftFinal);
		this.driveRightSide(rightFinal);
	}

	public boolean detectCollisions() {
		boolean collisionDetected = false;

		double currentAccelerationX = orientationGyro.getWorldLinearAccelX();
		double currentAccelerationY = orientationGyro.getWorldLinearAccelY();

		double currentJerkX = currentAccelerationX - _lastAccelerationX;
		double currentJerkY = currentAccelerationY - _lastAccelerationY;

		_lastAccelerationX = currentAccelerationX;
		_lastAccelerationY = currentAccelerationY;

		if (currentJerkX > _highestJerkX) {
			_highestJerkX = currentJerkX;
		}

		if (currentJerkY > _highestJerkY) {
			_highestJerkY = currentJerkY;
		}

		if (Math.abs(currentJerkX) > Calibrations.DRIVE_TRAIN_COLLISION_JERK_THRESHOLD) {
			collisionDetected = true;
		}

		if (Math.abs(currentJerkY) > Calibrations.DRIVE_TRAIN_COLLISION_JERK_THRESHOLD) {
			collisionDetected = true;
		}

		return collisionDetected;
	}

	public void outputJerk() {
		double currentAccelerationX = orientationGyro.getWorldLinearAccelX();
		double currentAccelerationY = orientationGyro.getWorldLinearAccelY();

		double currentJerkX = currentAccelerationX - _lastAccelerationX;
		double currentJerkY = currentAccelerationY - _lastAccelerationY;

		System.out.println("X Jerk: " + currentJerkX + " Y Jerk: " + currentJerkY);
	}

	public void outputHighestJerk() {
		System.out.println("Highest X Jerk: " + _highestJerkX + " Highest Y Jerk: " + _highestJerkY);
	}

	public void stopAndWait() {
		enableAutomatedDriving(0);
	}

	public void turnRelativeDegrees(double degrees) {
		this.setGyroTargetHeading(this.gyroTargetHeading + degrees);
	}

	public void enableAutomatedDriving(int direction, double speed) {
		automatedDrivingDirection = direction;
		enableAutomatedDriving(speed);
	}

	public void enableAutomatedDriving(double speed) {
		automatedDrivingEnabled = true;
		automatedDrivingSpeed = speed;
	}

	public void overrideAutomatedDriving() {
		// Just disable all the automated driving variables, and
		// the normal drive function will immediately resume.
		automatedDrivingEnabled = false;
		drivingThroughObstacle = false;
		hasHitObstacle = false;
		turning = false;
		waiting = false;
	}

	public void stop() {
		this.fpsTankManual(0, 0);
	}

	public void gyroStop() {
		this.setGyroTargetHeadingToCurrentHeading();
		this.resetGyroAdjustmentScaleFactor();
	}

	public boolean automatedActionHasCompleted() {
		// Just return the opposite of automatedDrivingEnabled.
		return automatedDrivingEnabled == false;
	}

	public void maintainStateWaiting() {
		this.stop();
	}

	public void wake() {
		this.waiting = false;
		this.automatedDrivingEnabled = false;
	}

	public void maintainStateTurning() {
		if (Math.abs(gyroTargetHeading - getCurrentHeading()) < 3) {
			automatedDrivingEnabled = false;
			turning = false;
		}
	}

	// GETTERS AND SETTER/RESET METHODS






	// Adjust the turn value by performing the following operations:
	// 1. Adjust the input value such that it is a percentage of the non-deadband input range, not the total input range.
	// To do this, calculate the non-deadband range, and then subtract the deadband value from the input.
	// 2. Square the adjusted input value while keeping its sign intact.
	// 3. Calculate the "moveable range", which is the range of output that will break static friction.
	// 4. Apply the adjusted input percentage to the moveable range.
	// 5. Add the feedforward value to the moveable range input percentage.
	public double getAdjustedTurnValue(double turn) {
		double inputRange = 1 - Calibrations.deadbandMagnitude;

		double deadbandDifference = Calibrations.deadbandMagnitude;
		if (turn < 0) {
			deadbandDifference *= -1;
		}

		double inputMinusDeadband = turn - deadbandDifference;
		double percentOfInputRange = inputMinusDeadband / inputRange;
		double squaredPercentOfInputRange = Math.copySign(Math.pow(percentOfInputRange, 2), percentOfInputRange);
		
		double moveableRange = 1 - Calibrations.TURN_FEED_FORWARD_MAGNITUDE;
		double squaredInputPercentageOfMoveableRange = squaredPercentOfInputRange * moveableRange;

		double ffDifference = Calibrations.TURN_FEED_FORWARD_MAGNITUDE;
		if (turn < 0) {
			ffDifference *= -1;
		}

		double ffAdjustedInput = squaredInputPercentageOfMoveableRange + ffDifference;
/*
		System.out.print("Turn vals: turn: " + (double) Math.round(turn * 100) / 100);
		System.out.print(" in-DB: " + (double) Math.round(inputMinusDeadband * 100) / 100);
		System.out.print(" %ofIR: " + (double) Math.round(percentOfInputRange * 100) / 100);
		System.out.print(" sq%ofIR: " + (double) Math.round(squaredPercentOfInputRange * 100) / 100);
		System.out.print(" MR: " + (double) Math.round(moveableRange * 100) / 100);
		System.out.print(" sq%ofMR: " + (double) Math.round(squaredInputPercentageOfMoveableRange * 100) / 100);
		System.out.println(" ffAdj: " + (double) Math.round(ffAdjustedInput * 100) / 100);
*/

		if (turn == 0) {
			ffAdjustedInput = 0;
		}
		return ffAdjustedInput;
	}

	public double getFedForwardDriveValue(double input, double feedForward, boolean tuning) {
		double inputRange = 1 - Calibrations.deadbandMagnitude;

		double deadbandDifference = Calibrations.deadbandMagnitude;
		if (input < 0) {
			deadbandDifference *= -1;
		}

		double inputMinusDeadband = input - deadbandDifference;
		double percentOfInputRange = inputMinusDeadband / inputRange;
		double squaredPercentOfInputRange = Math.copySign(Math.pow(percentOfInputRange, 2), percentOfInputRange);
		
		double moveableRange = 1 - feedForward;
		double squaredInputPercentageOfMoveableRange = squaredPercentOfInputRange * moveableRange;

		double ffDifference = feedForward;
		if (input < 0) {
			ffDifference *= -1;
		}

		double ffAdjustedInput = squaredInputPercentageOfMoveableRange + ffDifference;

		if (tuning) {
			System.out.print("Turn vals: turn: " + (double) Math.round(input * 100) / 100);
			System.out.print(" in-DB: " + (double) Math.round(inputMinusDeadband * 100) / 100);
			System.out.print(" %ofIR: " + (double) Math.round(percentOfInputRange * 100) / 100);
			System.out.print(" sq%ofIR: " + (double) Math.round(squaredPercentOfInputRange * 100) / 100);
			System.out.print(" MR: " + (double) Math.round(moveableRange * 100) / 100);
			System.out.print(" sq%ofMR: " + (double) Math.round(squaredInputPercentageOfMoveableRange * 100) / 100);
			System.out.println(" ffAdj: " + (double) Math.round(ffAdjustedInput * 100) / 100);
		}

		if (input == 0) {
			ffAdjustedInput = 0;
		}
		return ffAdjustedInput;
	}

	public double getScaledTurnFromTranslation(double translation, double turn) {
		double turnScaleReduction = Calibrations.TRANSLATION_MAX_TURN_SCALING * Math.abs(translation);
		double turnCoefficient = 1 - turnScaleReduction;
		double netTurn = turn * turnCoefficient;

		return netTurn;
	}

	public double getDriveGyro() {
		return orientationGyro.getAngle();
	}

	public double getGyroTargetHeading() {
		return this.gyroTargetHeading;
	}

	public double setGyroTargetHeadingToCurrentHeading() {
		this.gyroTargetHeading = getCurrentHeading();
		return gyroTargetHeading;
	}

	public double setGyroTargetHeading(double angle) {
		this.gyroTargetHeading = angle;
		return gyroTargetHeading;
	}

	private boolean adjustGyroDueToTimer() {
		double time = this._gyroCooldownTimer.get();

		boolean adjust = false;

		if (time > 0 && time < Calibrations.GYRO_COOLDOWN_TIMER_TIME) {
			adjust = true;
		} else if (time > Calibrations.GYRO_COOLDOWN_TIMER_TIME) {
			_gyroCooldownTimer.stop();
		}

		return adjust;
	}

	public double getTurnableGyroAdjustment(double turn) {
		// If the gyro is in disabled mode, just return immediately.
		if (gyroMode == Calibrations.GYRO_DISABLED) {
			return 0;
		}

		if (Math.abs(turn) > 0 || this.adjustGyroDueToTimer()) {
			this.setGyroTargetHeadingToCurrentHeading();

			if (Math.abs(turn) > 0) {
				this._gyroCooldownTimer.reset();
				this._gyroCooldownTimer.start();
			}
		}

		return getStaticGyroAdjustment();
	}

	public double getStaticGyroAdjustment() {
		// If the gyro is in disabled mode, just return immediately.
		if (gyroMode == Calibrations.GYRO_DISABLED) {

			return 0;
		}

		double heading = getCurrentHeading();

		// Mod to eliminate extra rotations.
		double gyroAdjust = (heading - gyroTargetHeading) % 360;

		if (gyroAdjust < 0) {
			gyroAdjust = 360 + gyroAdjust;
		}

		// This snippet ensures that the robot will spin in the fastest direction to
		// zero
		// if it ends up more than 180 degrees off of intention.
		if (gyroAdjust > 180) {
			gyroAdjust = gyroAdjust - 360;
		}

		if (gyroAdjust > 180 || gyroAdjust < -180) {
			gyroAdjust *= -1;
		}

		// Mod again in case the directional snippet was applied.
		gyroAdjust = Math.round(gyroAdjust) % 360;

		gyroAdjust *= _gyroAdjustmentScaleFactor;

		return gyroAdjust;
	}

	public double getCurrentHeading() {
		double heading = orientationGyro.getAngle();

		heading = heading % 360;

		if (heading < 0) {
			heading += 360;
		}

		return heading;
	}

	public double getPitchAngle() {
		return orientationGyro.getPitch();
	}

	public double getRollAngle() {
		return orientationGyro.getRoll();
	}

	public double getAvgNetInchesTraveled() {
		return (getRightNetInchesTraveled() + getLeftNetInchesTraveled()) / 2;
	}

	public double getRightNetInchesTraveled() {
		double rightNetRevolutions = driveRight.getEncoderPosition() / Calibrations.talonSRXMotorTicksPerRevolution;
		
		return rightNetRevolutions * Calibrations.wheelCircumferenceInches;
	}
	
	public double getLeftNetInchesTraveled() {
		double leftNetRevolutions = driveLeft.getEncoderPosition() / Calibrations.talonSRXMotorTicksPerRevolution;

		return -leftNetRevolutions * Calibrations.wheelCircumferenceInches;
	}

	public void setSlewRate(double slewRate) {
		_slewRate = slewRate;
		driveRight.setSlewRate(_slewRate);
		driveLeft.setSlewRate(_slewRate);
	}

	public void setMaxPower(double maxPower) {
		driveRight.setMaxPower(maxPower);
		driveLeft.setMaxPower(maxPower);
	}

	public void resetDriveEncoders() {
		driveLeft.resetEncoderPosition();
		driveRight.resetEncoderPosition();
	}

	public void setDriveMode(int driveMode) {
		this.driveMode = driveMode;
	}

	public void setCutPower(boolean cutPower) {
		_cutPower = cutPower;
	}

	public boolean getCutPower() {
		return _cutPower;
	}

	public void setGyroMode(int gyroMode) {
		this.gyroMode = gyroMode;
	}

	public double getGyroAdjustmentScaleFactor() {
		return _gyroAdjustmentScaleFactor;
	}

	public void setGyroAdjustmentScaleFactor(double gyroAdjustmentScaleFactor) {
		_gyroAdjustmentScaleFactor = gyroAdjustmentScaleFactor;
	}

	public void resetGyroAdjustmentScaleFactor() {
		setGyroAdjustmentScaleFactor(Calibrations.GYRO_ADJUSTMENT_DEFAULT_SCALE_FACTOR);
	}

	public double getSlewRate() {
		return _slewRate;
	}

	public void resetOrientationGyro() {
		orientationGyro.reset();
	}

	public double getGyroAngle() {
		return orientationGyro.getAngle();
	}
}

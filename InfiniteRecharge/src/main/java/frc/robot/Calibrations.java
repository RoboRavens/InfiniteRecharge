package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class Calibrations {

    // DRIVE TRAIN
	// Slew rate of .2 seems to work well for when the lift is lowered, though more testing
	// is necessary - might turn it up or down slightly for increased performance.
	// public static final double slewRate = .2;
	public static final double slewRateMinimum = .3;
	public static final double slewRateMaximum = .35;
	
	// The safe slew rate changes based upon a few variables:
	// 		- What gear we are in
	//		- How high the lift is
	//		- What direction the robot is moving (forward or backward.)
	//			(backwards to forwards seems worse - but that's with no arm or cube, and a broken chassis)
	//		- A number low enough to be safe for all scenarios will negatively impact normal operation.
	
	public static final double cutPowerModeMovementRatio = .3;
	public static final double cutPowerModeTurnRatio = .5;
	public static final double gyroAdjustmentDefaultScaleFactor = .025;
	public static final double driveTrainTurnRelativeDegreesGyroAdjustmentScaleFactor = .03;
	public static final double gyroCooldownTimerTime = .5;
	public static final double translationMaxTurnScaling = .5;
	public static final double gyroAutoTurnAcceptableErrorDegrees = 1;
	public static final boolean driveTrainStartingIsInHighGear = false;
	

	public static final double turnFeedForwardMagnitude = .18;
	public static final double translationFeedForwardMagnitude = .1;


	// .35 is for minibot
	// public static final double turnFeedForwardMagnitude = .35;

	// .18 is for minibot
	// public static final double translationFeedForwardMagnitude = .18;
	
	// Drive collision
	public static final double DriveTrainCollisionJerkThreshold = 4;

	// 2019 and newer robots use talonSRX instead talon
	public static final Boolean UseTalonSRXForDriveController = true;

	// Drive and gyro modes
	public static final int bulldozerTank = 0;
	public static final int fpsTank = 1;
	
	public static final int gyroDisabled = 0;
	public static final int gyroEnabled = 1;
	
	// Any turn taking too long to complete (e.g. wheel scrub has halted the turn) will abandon after this number of seconds.
	public static final double DriveTrainTurnRelativeDegreesSafetyTimerSeconds = 1;
	public static final double DriveTrainDriveInchesSafetyTimerSeconds = 3;
	
	// Deadband
	public static final double deadbandMagnitude = .2;
	
	// Default drive and gyro modes
	public static final int defaultDriveMode = Calibrations.fpsTank;
	public static final int defaultGyroMode = Calibrations.gyroEnabled;
	
	
	// DRIVE ENCODERS
	public static final double encoderCUI103CyclesPerRevolution = 4096;
	public static final double talonSRXMotorTicksPerRevolution = 8186;
	public static final double wheelDiameterInches = 4;
	public static final double wheelCircumferenceInches = Calibrations.wheelDiameterInches * Math.PI;
	public static final double wheelDiameterFeet = 1/3;
	public static final double wheelCircumferenceFeet = Calibrations.wheelDiameterFeet * Math.PI;
	
	// We're using CUI 103 encoders on both sides of the drivetrain.
	public static final double encoderCyclesPerRevolution = talonSRXMotorTicksPerRevolution;

	// Encoder usage choice in case of one side breaking
	public static final int useLeftEncoderOnly = 0;
	public static final int useRightEncoderOnly = 1;
	public static final int useBothEncoders = 2;

	public static final int useWhichEncoders = useBothEncoders;
	
	// Direction magic numbers
	public static final int drivingForward = -1;
	public static final int drivingBackward = 1;
	
	// Adjust max power based on elevator height
	public static final double DRIVETRAIN_MAXPOWER_AT_MAX_ELEVEATOR_HEIGHT = .4;

	// Robot characterization generated values
	public static final double ksVolts = 0.12;
    public static final double kvVoltSecondsPerMeter = 2.27;
	public static final double kaVoltSecondsSquaredPerMeter = 0.09;
	public static final double kTrackwidthMeters = 0.5761920811967401;
	public static final double kMaxSpeedMetersPerSecond = 1.5;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;
	public static final double kPDriveVel = 0.00005;
    public static final double kIDriveVel = 0;
    public static final double kDDriveVel = 0.000178;

	// CLIMBER 
	public static final double climberHoldPositionPowerMagnitude = .13;
	public static final double climberExtendPowerMagnitude = .66;
	public static final double climberRetractPowerMagnitude = .4;
	
	public static final int climberEncoderMinimumValue = 0;
    public static final int climberEncoderMaximumValue = 53000;

    // The safety margin is how far away from the end of travel the encoders will stop the lift.
	// At low speeds (max of .3), and a lift max value of 30k, 1500 maxes out the climber.
	// At higher speeds, a higher value is needed because the climber will overshoot the target until we have PID.
	
	public static final int climberLiftUpwardSafetyMargin = 400;
	public static final int climberLiftDownwardSafetyMargin = 500;
	public static final int CLIMBER_AT_POSITION_BUFFER = 500;
	
	public static final double climberConsideredMovingEncoderRate = 0;
	
    public static final double CLIMBER_MOVE_TO_POSITION_TIMEOUT = 2;
    public static final double CLIMBER_SAFETY_TIMER_TIMEOUT = 5;
	
	public static final int climberInchesToEncoderTicksConversionValue = 411;
	public static final int climberInchesToEncoderTicksOffsetValue = 10;
	
	public static final int maximumTiltAngleWhileClimbing = 4;

	// HARVESTER
	public static final double harvesterPowerMagnitude = 1;
	public static final double AXIS_IS_PRESSED_VALUE = .25;

	// SHOOTER 
	public static final double shooterkF = 0.0;
    public static final double shooterkP = 0.7;
    public static final double shooterkI = 0.0005;
	public static final double shooterkD = 0.0075;
	public static final double targetRange = 100;
	
	// Velocity 600 = about 45 RPM (measured empirically)
    // 25 revolutions = 204661 encoder ticks (measured empirically)
    // 1 rev = ~8186 ticks

    // Velocity: measured in "change in native units per 100 ms"

    // 45 rpm = ~368,370 ticks
    // 60 seconds = 600 100ms segments
    // 600 * 600 = 360,000 ticks, so this checks out within range of measurement
    // error
    // 1 rpm = 8186 ticks in 600 time units
    // 1 rpm = 8192 / 600 = 13.64 velocity

	public static final double velToRpm = 8192 / 600;
	public static final double RPMToVel = 600 / 8192;

    /**
     * Convert 500 RPM to units / 100ms. 4096 Units/Rev * 500 RPM / 600 100ms/min in
     * either direction: velocity setpoint is in units/100ms
     */

	
	// LIMELIGHT
	public static final double FLOOR_TO_LIMELIGHT_LENS_HEIGHT = 19.5;
	public static final double FLOOR_TO_TARGET_CENTER_HEIGHT = 28.0;
	public static final double CAMERA_ANGLE_OFFSET_FROM_HORIZONTAL = 2; // degrees
	public static final double MINIMUM_DISTANCE_FROM_LIMELIGHT = 46.0;
	public static final double MAXIMUM_DISTANCE_FROM_LIMELIGHT = 240.0;
	public static final double LIMELIGHT_LENS_TO_ROBOT_CENTER_OFFSET_INCHES = 6.25;
	public static final int desiredTargetBuffer = 16;

	// LIGHTING
	public static final double lightingFlashTotalDurationMs = 1000;
	public static final double lightingFlashes = 10;

	//n CAMERA QUALITY
	public static final int cameraQuality = 50;

	// CONTROLLER RUMBLE
	public static final double gamePieceCollectedRumbleSeconds = .25;
}


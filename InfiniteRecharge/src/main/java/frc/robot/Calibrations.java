package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class Calibrations {

	// DRIVE TRAIN
	// Slew rate of .2 seems to work well for when the lift is lowered, though more
	// testing
	// is necessary - might turn it up or down slightly for increased performance.
	// public static final double slewRate = .2;
	public static final double SLEW_RATE_MINIMUM = .08;
	public static final double SLEW_RATE_MAXIMUM = .08;

	// The safe slew rate changes based upon a few variables:
	// - What gear we are in
	// - How high the lift is
	// - What direction the robot is moving (forward or backward.)
	// (backwards to forwards seems worse - but that's with no arm or cube, and a
	// broken chassis)
	// - A number low enough to be safe for all scenarios will negatively impact
	// normal operation.

	public static final double CUT_POWER_MODE_MOVEMENT_RATIO = .3;
	public static final double CUT_POWER_MODE_TURN_RATIO = .5;
	public static final double GYRO_ADJUSTMENT_DEFAULT_SCALE_FACTOR = .02;
	public static final double DRIVE_TRAIN_TURN_RELATIVE_DEGREES_GYRO_ADJUSTMENT_SCALE_FACTOR = .0015;
	public static final double GYRO_COOLDOWN_TIMER_TIME = .5;
	public static final double TRANSLATION_MAX_TURN_SCALING = .5;
	public static final double GYRO_AUTO_TURN_ACCEPTABLE_ERROR_DEGREES = 3;
	public static final boolean DRIVE_TRAIN_STARTING_IS_IN_HIGH_GEAR = false;

	public static final double TURN_FEED_FORWARD_MAGNITUDE = .18;
	public static final double TRANSLATION_FEED_FORWARD_MAGNITUDE = .1;

	// .35 is for minibot
	// public static final double turnFeedForwardMagnitude = .35;

	// .18 is for minibot
	// public static final double translationFeedForwardMagnitude = .18;

	// Drive collision
	public static final double DRIVE_TRAIN_COLLISION_JERK_THRESHOLD = 4;

	// 2019 and newer robots use talonSRX instead talon
	public static final Boolean USE_TALON_SRX_FOR_DRIVE_CONTROLLER = true;

	// Drive and gyro modes
	public static final int BULLDOZER_TANK = 0;
	public static final int FPS_TANK = 1;

	public static final int GYRO_DISABLED = 0;
	public static final int GYRO_ENABLED = 1;

	// Any turn taking too long to complete (e.g. wheel scrub has halted the turn)
	// will abandon after this number of seconds.
	public static final double DRIVE_TRAIN_TURN_RELATIVE_DEGREES_SAFETY_TIMER_SECONDS = 1;
	public static final double DRIVE_TRAIN_DRIVE_INCHES_SAFETY_TIMER_SECONDS = 3;

	// Deadband
	public static final double DEAD_BAND_MAGNITUDE = .2; 

	// Default drive and gyro modes
	public static final int DEFAULT_DRIVE_MODE = Calibrations.FPS_TANK;
	public static final int DEFAULT_GYRO_MODE = Calibrations.GYRO_ENABLED;

	// DRIVE ENCODERS
	public static final double ENCODER_CUI103_CYCLES_PER_REVOLUTION = 4096;
	public static final double TALON_SRX_MOTOR_TICKS_PER_REVOLUTION = 8186;
	public static final double TALON_FX_TICKS_PER_REVOLUTION = 2048;
	public static final double WHEEL_DIAMETER_INCHES = 4;
	public static final double WHEEL_CIRCUMFERENCE_INCHES = Calibrations.WHEEL_DIAMETER_INCHES * Math.PI;
	public static final double WHEEL_DIAMETER_FEET = 1 / 3;
	public static final double WHEEL_CIRCUMFERENCE_FEET = Calibrations.WHEEL_DIAMETER_FEET * Math.PI;

	// We're using CUI 103 encoders on both sides of the drivetrain.
	public static final double ENCODER_CYCLES_PER_REVOLUTION = TALON_SRX_MOTOR_TICKS_PER_REVOLUTION;

	// Encoder usage choice in case of one side breaking
	public static final int USE_LEFT_ENCODER_ONLY = 0;
	public static final int USE_RIGHT_ENCODER_ONLY = 1;
	public static final int USE_BOTH_ENCODERS = 2;

	public static final int USE_WHICH_ENCODERS = USE_BOTH_ENCODERS;

	// Direction magic numbers
	public static final int DRIVING_FORWARD = -1;
	public static final int DRIVING_BACKWARD = 1;

	// Adjust max power based on elevator height
	public static final double DRIVETRAIN_MAXPOWER_AT_MAX_ELEVEATOR_HEIGHT = .4;

	// Robot characterization generated values
	public static final double KS_VOLTS = 0.327;
	public static final double KV_VOLT_SECONDS_PER_METER = 2.26;
	public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.25;
	public static final double KP_DRIVE_VELOCITY = 0.01; //0.00005
	public static final double KI_DRIVE_VELOCITY = 0;
	public static final double KD_DRIVE_VELOCITY = 0;

	public static final double METERS_TO_INCHES = 39.37;

	public static final double TRACK_WIDTH_METERS = 0.7723672633409093;
	public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
			Calibrations.TRACK_WIDTH_METERS);
	public static final int ENCODER_CPR = 2048;
	public static final double WHEEL_DIAMETER_METERS = 0.15621;
	public static final double POST_ENCODER_GEARING = 9.77;
	public static final double ENCODER_DISTANCE_PER_PULSE =
			// Assumes the encoders are directly mounted on the wheel shafts
			((WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR) / POST_ENCODER_GEARING;

	public static final double MAX_SPEED_METERS_PER_SECOND = 1.5; // 1.5
	public static final double MAX_ACCELERATION_METERS_PER_SECOND = 3; // 3

	// Reasonable baseline values for a RAMSETE follower in units of meters and
	// seconds
	public static final double RAMSETE_B = 2;
	public static final double RAMSETE_ZETA = 0.7;

	// CLIMBER
	public static final double CLIMBER_HOLD_POSITION_POWER_MAGNITUDE = 0; //.13
	public static final double CLIMBER_EXTEND_POWER_MAGNITUDE = 1;
	public static final double CLIMBER_RETRACT_POWER_MAGNITUDE = -.4;
	public static final double CLIMBER_RETRACT_TO_LATCH_POWER_MAGNITUDE = .2;

	public static final int CLIMBER_ENCODER_MINIMUM_VALUE = 0;
	public static final int CLIMBER_ENCODER_MAXIMUM_VALUE = 53000;

	// The safety margin is how far away from the end of travel the encoders will
	// stop the lift.
	// At low speeds (max of .3), and a lift max value of 30k, 1500 maxes out the
	// climber.
	// At higher speeds, a higher value is needed because the climber will overshoot
	// the target until we have PID.

	public static final int CLIMBER_LIFT_UPWARD_SAFETY_MARGIN = 400;
	public static final int CLIMBER_LIFT_DOWNWARD_SAFETY_MARGIN = 500;
	public static final int CLIMBER_AT_POSITION_BUFFER = 500;

	public static final double CLIMBER_CONSIDERED_MOVING_ENCODER_RATE = 0;

	public static final double CLIMBER_MOVE_TO_POSITION_TIMEOUT = 2;
	public static final double CLIMBER_SAFETY_TIMER_TIMEOUT = 5;

	public static final int CLIMBER_INCHES_TO_ENCODER_TICKS_CONVERSION_VALUE = 411;
	public static final int CLIMBER_INCHES_TO_ENCODER_TICKS_OFFSET_VALUE = 10;

	public static final int MAXIMUM_TILT_ANGLE_WHILE_CLIMBING = 4;

	// INTAKE
	public static final double INTAKE_COLLECT_POWER_MAGNITUDE = .5; // 1
	public static final double INTAKE_SPIT_POWER_MAGNITUDE = -1;
	public static final double AXIS_IS_PRESSED_VALUE = .25;

	// HOPPER
	public static final double HOPPER_LEFT_FORWARD = 0.5;
	public static final double HOPPER_RIGHT_FORWARD = 0.5;
	public static final double HOPPER_LEFT_REVERSE = -0.5;
	public static final double HOPPER_RIGHT_REVERSE = -0.5;
	public static final double HOPPER_FEED_FULL_SPEED_LEFT = 1.0;
	public static final double HOPPER_FEED_FULL_SPEED_RIGHT = -1.0;
	public static final double HOPPER_STOP = 0;

	// CONVEYANCE
	public static final double CONVEYANCE_FULL_SPEED = -.25;
	public static final double CONVEYANCE_FULL_SPEED_REVERSE = 0.25; // 1;
	public static final double CONVEYANCE_NORMAL_SPEED = -0.5;
	public static final double CONVEYANCE_NORMAL_REVERSE_SPEED = 0.5;
	public static final double CONVEYANCE_FEEDER_SPEED = .25;
	public static final double CONVEYANCE_FEEDER_STOP = 0.0;
	public static final double CONVEYANCE_REVERSE_FEEDER = -.25;
	public static final double CONVEYANCE_REVERSE_FEEDER_SLOW = -.18;
	public static final double CONVEYANCE_STOP = 0;
	public static final double CONVEYANCE_SAFETY_TIMER_TIMEOUT = 5;
	public static final double CONVEYANCE_FEEDER_SPEED_SLOW = -.2;


	// SHOOTER 
	public static final double INIT_LINE_RPM = 4600; // actual is 4600
	public static final double I_SHOOTER_KF = 0.0085;
    public static final double I_SHOOTER_KP = 0.06; 
    public static final double I_SHOOTER_KI = 0.0;
	public static final double I_SHOOTER_KD = 0.0;

	public static final double CLOSE_TRENCH_SHOT_RPM = 5140; // actual is 5140
	public static final double C_SHOOTER_KF = 0.0085;
    public static final double C_SHOOTER_KP = 0.06; 
    public static final double C_SHOOTER_KI = 0.0;
	public static final double C_SHOOTER_KD = 0.0;

	public static final double FAR_TRENCH_SHOT_RPM = 0;
	public static final double F_SHOOTER_KF = 0.0085;
    public static final double F_SHOOTER_KP = 0.06; 
    public static final double F_SHOOTER_KI = 0.0;
	public static final double F_SHOOTER_KD = 0.0;
	
	public static final double VEL_TO_RPM = 8192 / 600;
	public static final double RPM_TO_VEL = 1 / VEL_TO_RPM;
	public static final double TARGET_RPM_BUFFER = 200;

	//Current Limiting
	public static final int LIMIT_DRIVE_AMPS = 10;
	public static final int CONVEYANCE_FEEDER_LIMIT = 40;
	public static final int TIMEOUT = 0;
	
	/*

	sets soft and hard limits for Falcon Motor RPM
	THE SOFT VALUES NEED TO BE CHANGED, they are placeholders and will not opperate
	*/


	 // public static final double INIT_LINE_RPM = INIT_LINE_VELOCITY / VEL_TO_RPM;
	
	public static final int FALCON_RPM_HARD_MAX = 6380;
	public static final int FALCON_RPM_HARD_MIN = 0;
	public static final int FALCON_RPM_SOFT_MAX = 0;
	public static final int FALCON_RPM_SOFT_MIN = 0;

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
	public static final int DESIRED_TARGET_BUFFER = 3;

	// LIGHTING
	public static final double LIGHTING_FLASH_TOTAL_DURATION_MS = 1000;
	public static final double LIGHTING_FLASHES = 10;

	// n CAMERA QUALITY
	public static final int CAMERA_QUALITY = 50;

	// CONTROLLER RUMBLE
	public static final double GAME_PIECE_COLLECTED_RUMBLE_SECONDS = .25;
}

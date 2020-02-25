package frc.ravenhardware;

import frc.controls.AxisCode;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.RobotMap;

import java.util.List;
import java.util.stream.Collectors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

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
	protected int automatedDrivingDirection = Calibrations.DRIVING_FORWARD;
	protected double automatedDrivingSpeed = 0;

	protected boolean hasHitObstacle = false;
	protected boolean drivingThroughObstacle = false;
	protected boolean turning = false;
	protected boolean waiting = false;

	public double gyroAdjust;
	double _gyroAdjustmentScaleFactor = Calibrations.GYRO_ADJUSTMENT_DEFAULT_SCALE_FACTOR;

	public boolean userControlOfCutPower = true;

	IRavenTalon driveLeft = new RavenTalonFX(RobotMap.LEFT_DRIVE_CHANNEL_1, RobotMap.LEFT_DRIVE_CHANNEL_2, "MotorLeft", _slewRate, false);
	IRavenTalon driveRight = new RavenTalonFX(RobotMap.RIGHT_DRIVE_CHANNEL_1, RobotMap.RIGHT_DRIVE_CHANNEL_2, "MotorRight", _slewRate, true);

	private DifferentialDriveOdometry _odometry;

	public RavenTank() {
		initializeRavenTank();
	}

	private void initializeRavenTank() {
		_slewRate = Calibrations.SLEW_RATE_MAXIMUM;

		_gyroCooldownTimer = new Timer();

		_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeading()));

		setDriveMode(Calibrations.DEFAULT_DRIVE_MODE);
		setCutPower(false);

		setGyroMode(Calibrations.DEFAULT_GYRO_MODE);
		gyroTargetHeading = setGyroTargetHeadingToCurrentHeading();
	}

	public double deadband(double input) {
		double output = input;

		if (Math.abs(output) < Calibrations.DEAD_BAND_MAGNITUDE) {
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
		double squaredTranslation = getFedForwardDriveValue(translation,
				Calibrations.TRANSLATION_FEED_FORWARD_MAGNITUDE, false);
		// double squaredTranslation = Math.copySign(Math.pow(translation, 2),
		// translation);

		if (Robot.DRIVE_CONTROLLER.getAxis(AxisCode.LEFTTRIGGER) > .25) {
			Robot.LIMELIGHT_SUBSYSTEM.turnLEDOn();
			fpsTankChooseLimelightOrManual(squaredTranslation, adjustedTurn);
		} else {
			Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
			fpsTankManual(squaredTranslation, adjustedTurn);
		}
	}

	public void fpsTankChooseLimelightOrManual(double translation, double turn) {
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
	// 1. Adjust the input value such that it is a percentage of the non-deadband
	// input range, not the total input range.
	// To do this, calculate the non-deadband range, and then subtract the deadband
	// value from the input.
	// 2. Square the adjusted input value while keeping its sign intact.
	// 3. Calculate the "moveable range", which is the range of output that will
	// break static friction.
	// 4. Apply the adjusted input percentage to the moveable range.
	// 5. Add the feedforward value to the moveable range input percentage.
	public double getAdjustedTurnValue(double turn) {
		double inputRange = 1 - Calibrations.DEAD_BAND_MAGNITUDE;

		double deadbandDifference = Calibrations.DEAD_BAND_MAGNITUDE;
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
		 * System.out.print("Turn vals: turn: " + (double) Math.round(turn * 100) /
		 * 100); System.out.print(" in-DB: " + (double) Math.round(inputMinusDeadband *
		 * 100) / 100); System.out.print(" %ofIR: " + (double)
		 * Math.round(percentOfInputRange * 100) / 100); System.out.print(" sq%ofIR: " +
		 * (double) Math.round(squaredPercentOfInputRange * 100) / 100);
		 * System.out.print(" MR: " + (double) Math.round(moveableRange * 100) / 100);
		 * System.out.print(" sq%ofMR: " + (double)
		 * Math.round(squaredInputPercentageOfMoveableRange * 100) / 100);
		 * System.out.println(" ffAdj: " + (double) Math.round(ffAdjustedInput * 100) /
		 * 100);
		 */

		if (turn == 0) {
			ffAdjustedInput = 0;
		}
		return ffAdjustedInput;
	}

	public double getFedForwardDriveValue(double input, double feedForward, boolean tuning) {
		double inputRange = 1 - Calibrations.DEAD_BAND_MAGNITUDE;

		double deadbandDifference = Calibrations.DEAD_BAND_MAGNITUDE;
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

	public Pose2d getPose() {
		return _odometry.getPoseMeters();
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
		return driveRight.getDistanceMeters() * Calibrations.METERS_TO_INCHES;
	}

	public double getLeftNetInchesTraveled() {
		return driveLeft.getDistanceMeters() * Calibrations.METERS_TO_INCHES;
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

	private void resetDriveEncoders() {
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

	private void resetOrientationGyro() {
		orientationGyro.reset();
	}

	public double getGyroAngle() {
		return orientationGyro.getAngle();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from 180 to 180
	 */
	private double getHeading() {
		return Math.IEEEremainder(orientationGyro.getAngle(), 360) * -1;
	}

	public void updateOdometry() {
		_odometry.update(Rotation2d.fromDegrees(this.getHeading()), driveLeft.getDistanceMeters(),
				driveRight.getDistanceMeters());
	}

	public void resetOdometry() {
		this.resetDriveEncoders();
		this.resetOrientationGyro();
		//_odometry.resetPosition(new Pose2d(new Translation2d(3, 0), Rotation2d.fromDegrees(getHeading())), Rotation2d.fromDegrees(getHeading()));
		_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading()));
	}

	public void setOdemetry(Pose2d start) {
		// for now assume it starts at 0 rad heading
		_odometry.resetPosition(start, Rotation2d.fromDegrees(getHeading()));
	}

	public void tankDriveVolts(double left, double right) {
		driveLeft.setVoltage(left);
		driveRight.setVoltage(-right);
	}

	/**
	 * Returns the current wheel speeds of the robot.
	 *
	 * @return The current wheel speeds.
	 */
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(driveLeft.getRateMeters(), driveRight.getRateMeters());
	}

	public Command getCommandForTrajectory(Trajectory trajectory) {
		// this code would only run when the command is generated, not when the command is used... so I'm commenting it out for now
		// var transform = _odometry.getPoseMeters().minus(trajectory.getInitialPose());
		// trajectory = trajectory.transformBy(transform);

		RamseteCommand ramseteCommand = new RamseteCommand(trajectory, _odometry::getPoseMeters,
				new RamseteController(Calibrations.RAMSETE_B, Calibrations.RAMSETE_ZETA),
				new SimpleMotorFeedforward(Calibrations.KS_VOLTS, Calibrations.KV_VOLT_SECONDS_PER_METER,
						Calibrations.KA_VOLT_SECONDS_SQUARED_PER_METER),
				Calibrations.DRIVE_KINEMATICS, this::getWheelSpeeds,
				new PIDController(Calibrations.KP_DRIVE_VELOCITY, Calibrations.KI_DRIVE_VELOCITY,
						Calibrations.KD_DRIVE_VELOCITY),
				new PIDController(Calibrations.KP_DRIVE_VELOCITY, Calibrations.KI_DRIVE_VELOCITY,
						Calibrations.KD_DRIVE_VELOCITY),
				// RamseteCommand passes volts to the callback
				this::tankDriveVolts, Robot.DRIVE_TRAIN_SUBSYSTEM);

		// Run path following command, then stop at the end.
		return ramseteCommand.andThen(() -> {
			this.tankDriveVolts(0, 0);
		});
	}

	// Change a trajectory so that the robot follows it but drives backwards
	public Trajectory reverseTrajectory(Trajectory trajectory){
		final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));
		List<State> states = trajectory.getStates();
		return new Trajectory(states.stream().map(state -> 
		{
			var newPose = state.poseMeters.transformBy(flip);
			var newCurvatureRadPerMeter = state.curvatureRadPerMeter * -1;
			return new State(state.timeSeconds,
				state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq,
				newPose, newCurvatureRadPerMeter);
		})
        .collect(Collectors.toList()));
	}

	// Change a trajectory so that the robot follows it but drives backwards
	public Trajectory reverseTrajectory2(Trajectory trajectory){
		List<State> states = trajectory.getStates();
		var waypoints = states.stream().map(state -> 
		{
			return state.poseMeters;
		})
		.collect(Collectors.toList());

		return TrajectoryGenerator.generateTrajectory(waypoints, this.getTrajectoryConfig().setReversed(true));
	}

		// Change a trajectory so that the robot follows it but drives backwards
		public Trajectory reverseTrajectory3(Trajectory trajectory){
			List<State> states = trajectory.getStates();
			var start = new Pose2d(3, 0, new Rotation2d(0));
			var end = new Pose2d(0, 0, new Rotation2d(0));
			//var waypoints = states.stream().map(state -> 
			//{
			//	return new Translation2d(state.poseMeters.getTranslation().getX(), state.poseMeters.getTranslation().getY());
			//})
			//.collect(Collectors.toList());
	
			return TrajectoryGenerator.generateTrajectory(start, List.of(), end, this.getTrajectoryConfig().setReversed(true));
		}

	public TrajectoryConfig getTrajectoryConfig() {
		// Create a voltage constraint to ensure we don't accelerate too fast
		var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Calibrations.KS_VOLTS, Calibrations.KV_VOLT_SECONDS_PER_METER,
						Calibrations.KA_VOLT_SECONDS_SQUARED_PER_METER),
				Calibrations.DRIVE_KINEMATICS, 10);

		// Create config for trajectory
		return new TrajectoryConfig(Calibrations.MAX_SPEED_METERS_PER_SECOND,
				Calibrations.MAX_ACCELERATION_METERS_PER_SECOND)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(Calibrations.DRIVE_KINEMATICS)
						// Apply the voltage constraint
						.addConstraint(autoVoltageConstraint).setReversed(false);
	}

	public void logPose(){
		System.out.println("pose X||Y||ActualDegrees = " + getPose().getTranslation().getX() + "||" + getPose().getTranslation().getY() + "||" + getHeading());
	}
	  
	public void currentLimting() {
		//This is the FalconFX current limting
		Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.driveLeft.setCurrentLimit(Calibrations.LIMIT_DRIVE_AMPS);
		Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.driveRight.setCurrentLimit(Calibrations.LIMIT_DRIVE_AMPS);
	}
}

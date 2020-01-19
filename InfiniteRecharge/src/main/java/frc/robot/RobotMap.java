package frc.robot;

public class RobotMap {
	// MOTORS
	public static final int leftDriveChannel = 0;
	public static final int leftDriveChannel2 = 2;
	public static final int rightDriveChannel = 1;
	public static final int rightDriveChannel2 = 3;

	public static final int conveyanceMotor = 4;
	public static final int conveyanceMotor2 = 5;

	public static final int shooterMotor = 6;
	public static final int shooterMotor2 = 7;

	public static final int intakeMotor = 8;

	public static final int climberMotor = 9;
	public static final int climberMotor2 = 10;
	public static final int climberMotorBrake = 11;

	// PCM
	public static final int beakCaptureSolenoid = 0;
	public static final int beakReleaseSolenoid = 1;
	public static final int intakeExtendTransportSolenoid = 2;
	public static final int intakeRetractTransportSolenoid = 3;

	// DIO PORTS
	public static final int frontLineSensor = 0;
	public static final int rearLineSensor = 1;
	public static final int hatchPanelSensor = 2;
	public static final int cargoSensorLeft = 6;
	public static final int cargoSensorRight = 7;

	// Arm extension and retraction limit switches are deprecated.
	public static final int armExtensionLimitSwitch = 4;
	public static final int armRetractionLimitSwitch = 5;

	// RELAYS
	public static final int hasGamePieceRelay = 0;
	public static final int lineAlignmentRelay = 1;

	// CAMERA
	public static final String cameraName = "cam0";
}

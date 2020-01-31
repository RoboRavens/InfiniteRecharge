package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.BufferedValue;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.DriveTrainDriveInchesCommand;
import frc.robot.commands.drivetrain.DriveTrainStopCommand;
import frc.util.NetworkTableDiagnostics;

public class LimelightSubsystem extends SubsystemBase {
	static double Limit = 0.00;
	edu.wpi.first.networktables.NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry tx = table.getEntry("tx");
	NetworkTableEntry ty = table.getEntry("ty");
	NetworkTableEntry ta = table.getEntry("ta");
	NetworkTableEntry tv = table.getEntry("tv");
	NetworkTableEntry ledMode = table.getEntry("ledMode");

	private int _ledState = 3;

	private double _heightDifference = Calibrations.FLOOR_TO_TARGET_CENTER_HEIGHT - Calibrations.FLOOR_TO_LIMELIGHT_LENS_HEIGHT;
	private double _angleToTargetFromHorizontal = 0;
	private double _inchesToTarget = 0;
	private double _angleComplimenting90 = 0; // Math.acos does not accept fractions as a parameter so this variable was made
	private double _targetAngle = 0;
	private double _powerMagnitude = 0.0;
  	private double _distanceDesiredFromTarget = 0.0;
  	private double _distanceToDrive = 0.0;
	private int _direction = 0;
	private double _offsetFromTargetAngle = 0.0;

	DriveTrainDriveInchesCommand driveTrainDriveInchesCommand = new DriveTrainDriveInchesCommand(_distanceToDrive, _powerMagnitude, _direction);

	private BufferedValue bufferedAngleOffHorizontal = new BufferedValue(9);

	public LimelightSubsystem() {
		this.initialize();
	}

	public void initialize() {
		NetworkTableDiagnostics.SubsystemNumber("Limelight", "TargetArea", () -> this.getTargetArea());
		NetworkTableDiagnostics.SubsystemNumber("Limelight", "angleOffHorizontal", () -> this.angleOffHorizontal());
		NetworkTableDiagnostics.SubsystemNumber("Limelight", "angleOffVertical", () -> this.angleOffVertical());
		NetworkTableDiagnostics.SubsystemBoolean("Limelight", "hasTarget", () -> this.hasTarget());
		NetworkTableDiagnostics.SubsystemNumber("Limelight", "Vision Tracking Distance (Inches)", () -> _inchesToTarget);
		NetworkTableDiagnostics.SubsystemNumber("Limelight", "Height Difference", () -> _heightDifference);
		NetworkTableDiagnostics.SubsystemNumber("Limelight", "Angle From Crosshair to Target", () -> _angleToTargetFromHorizontal);
		NetworkTableDiagnostics.SubsystemNumber("Limelight", "TargetAngle", () -> _targetAngle);
		NetworkTableDiagnostics.SubsystemNumber("Limelight", "TargetAngleOffset", () -> _offsetFromTargetAngle);
		NetworkTableDiagnostics.SubsystemNumber("Limelight", "LED State", () -> _ledState);
	}

	public void periodic() {

		_angleToTargetFromHorizontal = Math.tan(Math.toRadians(Calibrations.CAMERA_ANGLE_OFFSET_FROM_HORIZONTAL + ty.getDouble(0.0)));
		_inchesToTarget = _heightDifference/_angleToTargetFromHorizontal;
		_angleComplimenting90 = Calibrations.LIMELIGHT_LENS_TO_ROBOT_CENTER_OFFSET_INCHES/_inchesToTarget;
		_targetAngle = 90 - Math.toDegrees(Math.acos(_angleComplimenting90));
		_targetAngle = _targetAngle - 4.2; // limelight was mounted at a slight angle, this is the correcting number
		
		if (this.hasTarget()) {
			_offsetFromTargetAngle = _targetAngle - angleOffHorizontal();
		} else {
			_offsetFromTargetAngle = 0;
		}
	
		bufferedAngleOffHorizontal.maintainState(this.angleOffHorizontal());
	}

	public double getTargetArea() {
		return ta.getDouble(0);
	}

	public boolean hasTarget() {
		boolean hasTarget;
		if (tv.getDouble(0) == 1) {
			hasTarget = true;
		} else {
			hasTarget = false;
		}
		return hasTarget;
	}

	public double angleOffHorizontal() {
		return tx.getDouble(0);
	}

	public double angleOffVertical() {
		return ty.getDouble(0);
	}

	public static void limeLightDiagnostics() {

	}

	public void turnToTarget() {
		Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.setGyroTargetHeading(Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCurrentHeading() + this.bufferedAngleOffHorizontal.getMedian());
	}

	public void driveToTarget(double distanceDesiredFromTarget) {
		_distanceDesiredFromTarget = distanceDesiredFromTarget;

		if (_inchesToTarget > (_distanceDesiredFromTarget + 18)) {
			_distanceToDrive = _inchesToTarget - _distanceDesiredFromTarget;
			_powerMagnitude = 0.6;
			_direction = Calibrations.drivingForward;
			CommandScheduler.getInstance().schedule(driveTrainDriveInchesCommand);
			System.out.println("MOVE FORWARD " + _distanceToDrive + " INCHES");

		} else if (_inchesToTarget < (_distanceDesiredFromTarget - 18)) {
			_distanceToDrive = _distanceDesiredFromTarget - _inchesToTarget;
			_powerMagnitude = 0.6;
			_direction = Calibrations.drivingForward;
			CommandScheduler.getInstance().schedule(driveTrainDriveInchesCommand);
			System.out.println("BACKING UP " + _distanceToDrive + " INCHES");

		} else if (this.hasTarget() == false) {
			CommandScheduler.getInstance().schedule(new DriveTrainStopCommand());
		} else {
			CommandScheduler.getInstance().schedule(new DriveTrainStopCommand());
			System.out.println("DO NOTHING, I'M AT 10 FEET");
		} 
	}

	public void toggleLED() {
		if (_ledState == 0) {
			setBothLEDOn();
		} else if (_ledState == 3) {
			setOneLEDOn();
		}
	}

	public void setOneLEDOn() {
		_ledState = 0;
	}

	public void setBothLEDOn() {
		_ledState = 3;
	}

	public void turnLEDOff() {
		ledMode.setNumber(1);
	}

	public void turnLEDOn() {
		ledMode.setNumber(_ledState);
	}

		/*Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.setGyroTargetHeading(Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCurrentHeading() + x);
        
		if (_inchesToTarget < Calibrations.MINIMUM_DISTANCE_FROM_LIMELIGHT) {
			_inchesToTarget = Calibrations.MINIMUM_DISTANCE_FROM_LIMELIGHT;
		}

		if (_inchesToTarget > Calibrations.MAXIMUM_DISTANCE_FROM_LIMELIGHT) {
			_inchesToTarget = Calibrations.MAXIMUM_DISTANCE_FROM_LIMELIGHT;
		}
		
		if (hasTarget()) {
			if (_inchesToTarget < (distanceDesiredFromTarget + Calibrations.desiredTargetBuffer) && _inchesToTarget > (distanceDesiredFromTarget - Calibrations.desiredTargetBuffer)) {
				(new DriveTrainDriveFPSCommand()).start();
				System.out.println("DO NOTHING, I'M AT 2 FEET");
			} else if (_inchesToTarget > distanceDesiredFromTarget) {
				DriveTrainDriveInchesCommand nick = new DriveTrainDriveInchesCommand(_inchesToTarget - distanceDesiredFromTarget, .2, Calibrations.drivingForward);
				nick.start();
				System.out.println("MOVE FORWARD " + (_inchesToTarget - distanceDesiredFromTarget) + " INCHES");
			} else if (_inchesToTarget < distanceDesiredFromTarget) {
				DriveTrainDriveInchesCommand nick = new DriveTrainDriveInchesCommand(distanceDesiredFromTarget - _inchesToTarget, .2, Calibrations.drivingBackward);
				nick.start();
				System.out.println("BACKING UP " + (distanceDesiredFromTarget - _inchesToTarget) + " INCHES");
			} 
		} else {
			
		*/

		/*driveTrainDriveInchesCommand.start();
    	
    	if (direction == Calibrations.drivingBackward) {
    	netInchesTraveledSoFar = distanceToDriveBlind - Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getNetInchesTraveled();
    	} else {
    	netInchesTraveledSoFar = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getNetInchesTraveled() - distanceToDriveBlind;
		   }*/
		   
		   /*  boolean hasTraveledTargetDistance = (netInchesTraveledSoFar >= distanceToDriveBlind); 
    	double area = ta.getDouble(0.0);
        
   	 	if (timeoutTimer.get() > timeoutSeconds) {
      	hasTraveledTargetDistance = true;

      	System.out.println("TIMEOUTTIMEOUTTIMEOUT");
    	}		

    	if (area > 0.0) {
      	new DriveDistanceToTargetCommand().start();
      	hasTraveledTargetDistance = true;
    	}
    	
    	return hasTraveledTargetDistance; */
}

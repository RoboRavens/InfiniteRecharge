package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.controls.AxisCode;
import frc.ravenhardware.RavenTank;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.util.NetworkTableDiagnostics;

public class DriveTrainSubsystem extends SubsystemBase {
	public RavenTank ravenTank;

	private double _maxPower;
	private double _slewRateFinal;

	public DriveTrainSubsystem() {
		ravenTank = new RavenTank();
		this.initialize();
	}

	public void defaultCommand() {
		double leftYAxisValue = Robot.DRIVE_CONTROLLER.getAxis(AxisCode.LEFTSTICKY);
		double rightYAxisValue = Robot.DRIVE_CONTROLLER.getAxis(AxisCode.RIGHTSTICKY);
		double rightXAxisValue = Robot.DRIVE_CONTROLLER.getAxis(AxisCode.RIGHTSTICKX);
		Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.drive(leftYAxisValue, rightYAxisValue, rightXAxisValue);
	}

	public void initialize() {
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "PowerMax", () -> _maxPower);
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "EncoderLeftInchesTraveled",
				() -> ravenTank.getLeftNetInchesTraveled());
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "EncoderRightInchesTraveled",
				() -> ravenTank.getRightNetInchesTraveled());
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "EncoderAvgInchesTraveled", () -> ((double) 0));
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "SlewRate", () -> _slewRateFinal);
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "PitchAngle", () -> ravenTank.getPitchAngle());
		NetworkTableDiagnostics.SubsystemBoolean("DriveTrain", "CutPower", () -> ravenTank.getCutPower());

	}

	public void periodic() {
		ravenTank.setMaxPower(1);

		SmartDashboard.putNumber("EncoderRightInchesTraveled", ravenTank.getRightNetInchesTraveled());
		SmartDashboard.putNumber("EncoderLeftInchesTraveled", ravenTank.getLeftNetInchesTraveled());
		ravenTank.logPose();

		double slewRate = Calibrations.SLEW_RATE_MAXIMUM;
		slewRate = Math.max(Calibrations.SLEW_RATE_MINIMUM, slewRate);
		_slewRateFinal = Math.min(Calibrations.SLEW_RATE_MAXIMUM, slewRate);
		ravenTank.setSlewRate(_slewRateFinal);
		ravenTank.updateOdometry();
	}
}

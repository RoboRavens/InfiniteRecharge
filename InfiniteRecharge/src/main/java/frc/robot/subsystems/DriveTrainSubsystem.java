package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ravenhardware.RavenTank;
import frc.robot.Calibrations;
import frc.robot.commands.drivetrain.DriveTrainDriveFPSCommand;
import frc.util.NetworkTableDiagnostics;

public class DriveTrainSubsystem extends SubsystemBase {
	public RavenTank ravenTank;

	private double _maxPower;
	private double _slewRateFinal;

	public DriveTrainSubsystem() {
		ravenTank = new RavenTank();

		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "PowerMax", () -> _maxPower);
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "EncoderLeftInchesTraveled", () -> ravenTank.getLeftNetInchesTraveled());
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "EncoderRightInchesTraveled", () -> ravenTank.getRightNetInchesTraveled());
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "EncoderAvgInchesTraveled", () -> ravenTank.getNetInchesTraveled());
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "SlewRate", () -> _slewRateFinal);
		NetworkTableDiagnostics.SubsystemNumber("DriveTrain", "PitchAngle", () -> ravenTank.getPitchAngle());
		NetworkTableDiagnostics.SubsystemBoolean("DriveTrain", "CutPower", () -> ravenTank.getCutPower());
	}

	public void initialize() {
		setDefaultCommand(new DriveTrainDriveFPSCommand());
	}

	public void periodic() {
		ravenTank.setMaxPower(1);

		double slewRate = Calibrations.slewRateMaximum;
		slewRate = Math.max(Calibrations.slewRateMinimum, slewRate);
		_slewRateFinal = Math.min(Calibrations.slewRateMaximum, slewRate);
		ravenTank.setSlewRate(_slewRateFinal);
	}
}

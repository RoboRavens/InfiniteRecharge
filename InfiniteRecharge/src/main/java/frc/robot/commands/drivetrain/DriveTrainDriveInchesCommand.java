package frc.robot.commands.drivetrain;

import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.util.PCDashboardDiagnostics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainDriveInchesCommand extends CommandBase {
	private double _powerMagnitude;
	private double _totalInchesToTravel;
	private double _driveTrainNetInchesTraveledAtStart;
	private double _netInchesTraveledSoFar = 0;
	private int _direction;
	private Timer _timeoutTimer;
	private double _timeoutSeconds = Calibrations.DriveTrainDriveInchesSafetyTimerSeconds;

	public DriveTrainDriveInchesCommand(double inchesToTravel, double powerMagnitude, int direction) {
		addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
		this._totalInchesToTravel = inchesToTravel;
		this._powerMagnitude = powerMagnitude *= direction;
		this._direction = direction;
		this._timeoutTimer = new Timer();
	}

	public DriveTrainDriveInchesCommand(double inchesToTravel, double powerMagnitude, int direction,
			double timeoutSeconds) {
		addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
		this._totalInchesToTravel = inchesToTravel;
		this._powerMagnitude = powerMagnitude *= direction;
		this._direction = direction;
		this._timeoutTimer = new Timer();
		this._timeoutSeconds = timeoutSeconds;
	}

	// Called just before this Command runs the first time
	public void initialize() {
		System.out.println("DriveTrainDriveInchesCommand init");
		System.out.println("RT NIT:" + (double) 0);
		_driveTrainNetInchesTraveledAtStart = (double) 0;
		_timeoutTimer.start();
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.fpsTankManual(_powerMagnitude, 0);

		if (_direction == Calibrations.drivingBackward) {
			_netInchesTraveledSoFar = _driveTrainNetInchesTraveledAtStart - (double) 0;
		} else {
			_netInchesTraveledSoFar = (double) 0 - _driveTrainNetInchesTraveledAtStart;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		boolean hasTraveledTargetDistance = (_netInchesTraveledSoFar >= _totalInchesToTravel);
		PCDashboardDiagnostics.AdHocNumber("netInchesTraveledSoFar", _netInchesTraveledSoFar);
		PCDashboardDiagnostics.AdHocNumber("totalInchesToTravel", _totalInchesToTravel);

		if (_timeoutTimer.get() > _timeoutSeconds) {
			hasTraveledTargetDistance = true;

			System.out.println("TIMEOUT TIMEOUT TIMEOUT TIMEOUT");
		}

		return hasTraveledTargetDistance;
	}
}

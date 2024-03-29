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
	private double _timeoutSeconds = Calibrations.DRIVE_TRAIN_DRIVE_INCHES_SAFETY_TIMER_SECONDS;

	public DriveTrainDriveInchesCommand(double inchesToTravel, double powerMagnitude, int direction) {
		addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
		_totalInchesToTravel = inchesToTravel;
		_powerMagnitude = powerMagnitude *= direction;
		_direction = direction;
		_timeoutTimer = new Timer();
	}

	public DriveTrainDriveInchesCommand(double inchesToTravel, double powerMagnitude, int direction,
			double timeoutSeconds) {
		addRequirements(Robot.DRIVE_TRAIN_SUBSYSTEM);
		_totalInchesToTravel = inchesToTravel;
		_powerMagnitude = powerMagnitude *= direction;
		_direction = direction;
		_timeoutTimer = new Timer();
		_timeoutSeconds = timeoutSeconds;
	}

	// Called just before this Command runs the first time
	public void initialize() {
		//System.out.println("DriveTrainDriveInchesCommand init");
		//System.out.println("RT NIT:" + Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getAvgNetInchesTraveled());
		_driveTrainNetInchesTraveledAtStart = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getAvgNetInchesTraveled();
		_timeoutTimer.start();
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.fpsTankManual(_powerMagnitude, 0);

		if (_direction == Calibrations.DRIVING_BACKWARD) {
			_netInchesTraveledSoFar = _driveTrainNetInchesTraveledAtStart
					- Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getAvgNetInchesTraveled();
		} else {
			_netInchesTraveledSoFar = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getAvgNetInchesTraveled()
					- _driveTrainNetInchesTraveledAtStart;
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
		}

		return hasTraveledTargetDistance;
	}
}

package frc.robot.commands.drivetrain;

import frc.ravenhardware.RavenTank;
import frc.robot.Calibrations;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainTurnRelativeDegreesCommand extends CommandBase {
    RavenTank ravenTank;
    Timer safetyTimer;

    // Negative degrees means turn left; positive means turn right.
    double degreesToTurn;
    double driveTrainOriginalHeading;
    double temporaryGyroScaleFactor;
    double previousGyroScaleFactor;
    private double _timeoutSeconds;

    public DriveTrainTurnRelativeDegreesCommand(DriveTrainSubsystem driveTrain, double degreesToTurn, double gyroScaleFactor, double timeoutSeconds) {
        addRequirements(driveTrain);
        this.ravenTank = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank;
        this.degreesToTurn = degreesToTurn;
        this.safetyTimer = new Timer();
        this.previousGyroScaleFactor = Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.getGyroAdjustmentScaleFactor();
        this.temporaryGyroScaleFactor = gyroScaleFactor;
        _timeoutSeconds = timeoutSeconds;
    }

    public DriveTrainTurnRelativeDegreesCommand(DriveTrainSubsystem driveTrain, double degreesToTurn, double gyroScaleFactor) {
        this(driveTrain, degreesToTurn, gyroScaleFactor, Calibrations.DriveTrainTurnRelativeDegreesSafetyTimerSeconds);
    }

    public DriveTrainTurnRelativeDegreesCommand(DriveTrainSubsystem driveTrain, double degreesToTurn) {
        this(driveTrain, degreesToTurn, Calibrations.driveTrainTurnRelativeDegreesGyroAdjustmentScaleFactor);
    }

    // Called just before this Command runs the first time
    public void initialize() {
        System.out.println("DriveTrainTurnRelativeDegreesCommand init");
        driveTrainOriginalHeading = ravenTank.getCurrentHeading();
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.setGyroAdjustmentScaleFactor(temporaryGyroScaleFactor);
        ravenTank.turnRelativeDegrees(degreesToTurn);
        safetyTimer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        ravenTank.fpsTankManual(0, 0);
    }

     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) {
        ravenTank.setGyroTargetHeadingToCurrentHeading();
        Robot.DRIVE_TRAIN_SUBSYSTEM.ravenTank.setGyroAdjustmentScaleFactor(previousGyroScaleFactor);
        ravenTank.stop();
     }
 
     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
        double currentHeading = ravenTank.getCurrentHeading();
        double degreesTurned = currentHeading - driveTrainOriginalHeading;

        double degreesAwayFromTarget = Math.abs(degreesToTurn - degreesTurned);
        boolean turnComplete = (degreesAwayFromTarget < Calibrations.gyroAutoTurnAcceptableErrorDegrees);

        if (safetyTimer.get() > _timeoutSeconds) {
            System.out.println("TIMEOUT TIMEOUT TIMEOUT TIMEOUT");
            turnComplete = true;
        }

        return turnComplete;
     }
}

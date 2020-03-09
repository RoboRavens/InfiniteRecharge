package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Calibrations;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * A command that will turn the robot to the specified angle.
 */
public class DriveTrainTurnPIDCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */
  public DriveTrainTurnPIDCommand(double targetAngleDegrees, DriveTrainSubsystem driveTrain) {
    super(
        new PIDController(Calibrations.KP_TURN, Calibrations.KI_TURN, Calibrations.KD_TURN), 
        // Close loop on heading
        () -> driveTrain.ravenTank.getHeading(),
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> driveTrain.ravenTank.tankDriveVolts(12, 12),
        // Require the drive
        driveTrain);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Calibrations.GYRO_AUTO_TURN_ACCEPTABLE_ERROR_DEGREES, Calibrations.GYRO_ADJUSTMENT_DEFAULT_SCALE_FACTOR);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.controls.ButtonCode;
import frc.controls.Gamepad;
import frc.controls.OperationPanel;
import frc.robot.commands.shooter.ControlPanelShotCommand;
import frc.robot.commands.shooter.InitiationLineShotCommand;
import frc.robot.commands.shooter.ShooterRevCommand;
import frc.robot.commands.shooter.ShooterRumbleFeedbackCommand;
import frc.robot.commands.shooter.ShooterTuneCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ProgrammableLEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.util.LoggerOverlord;
import frc.util.OverrideSystem;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "ReverseLine";
  private static final String kCustomAuto = "Curve";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public DriverStation driverStation;
  public PowerDistributionPanel PDP = new PowerDistributionPanel();

  public static final LoggerOverlord LOGGER_OVERLORD = new LoggerOverlord(1f);

  public static final Gamepad DRIVE_CONTROLLER = new Gamepad(0);
  public static final OperationPanel OPERATION_PANEL = new OperationPanel(1);

  public static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();
  public static final ConveyanceSubsystem CONVEYANCE_SUBSYSTEM = new ConveyanceSubsystem();
  public static final DriveTrainSubsystem DRIVE_TRAIN_SUBSYSTEM = new DriveTrainSubsystem();
  public static final HopperSubsystem HOPPER_SUBSYSTEM = new HopperSubsystem();
  public static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  public static final ProgrammableLEDSubsystem PROGRAMMABLE_LED_SUBSYSTEM = new ProgrammableLEDSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();

  public static final OverrideSystem OVERRIDE_SYSTEM_CLIMBER_EXTEND = new OverrideSystem();
  public static final OverrideSystem OVERRIDE_SYSTEM_CLIMBER_RETRACT = new OverrideSystem();

  public static boolean isRedAlliance;

  public String autoFromDashboard;
  public String positionFromDashboard;

  public ShooterRevCommand shooterRev = new ShooterRevCommand();
  public ControlPanelShotCommand shooterCtrlPanelShot = new ControlPanelShotCommand();
  public InitiationLineShotCommand shooterLineShot = new InitiationLineShotCommand();
  public ShooterTuneCommand shooterTune = new ShooterTuneCommand();

  public ShooterRumbleFeedbackCommand shooterRumble = new ShooterRumbleFeedbackCommand();

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    CLIMBER_SUBSYSTEM.resetEncodersToRetractedLimit();
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.resetDriveEncoders();
    INTAKE_SUBSYSTEM.retract();
    LIMELIGHT_SUBSYSTEM.turnLEDOff();
    this.setupDefaultCommands();
    this.setupShooterController();
    this.setupDriveController();
    this.setupOperationPanel();
  }

  private void setupDefaultCommands() {
    DRIVE_TRAIN_SUBSYSTEM
        .setDefaultCommand(new RunCommand(() -> DRIVE_TRAIN_SUBSYSTEM.defaultCommand(), DRIVE_TRAIN_SUBSYSTEM));
    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.tankDriveVolts(0, 0);
  }

  @Override
  public void autonomousInit() {
    Command autonomousCommand = null;

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      // List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
      List.of(new Translation2d(2, 0), new Translation2d(1, 1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass config
      DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig().setReversed(true)
    );

    //try {
      SmartDashboard.putString("PathFollowed", m_chooser.getSelected());
      //var trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/" + m_chooser.getSelected() + ".wpilib.json"));
      //DRIVE_TRAIN_SUBSYSTEM.ravenTank.reverseTrajectory(trajectory);
      autonomousCommand = DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(exampleTrajectory);
    //} catch (IOException e) {
      //e.printStackTrace();
    //}

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.setGyroTargetHeadingToCurrentHeading();

  }

  public void teleopPeriodic() {
  }

  public void setupShooterController() {
    System.out.println("DRIVE CONTROLLER CONFIGURED");
    System.out.println("Remember to enable to actually make things work");
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.A).whileHeld(shooterLineShot);
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.B).whileHeld(shooterTune);
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.X).whileHeld(shooterRev);
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.Y).whenPressed(shooterCtrlPanelShot);
  }

  public void setupOperationPanel() {
    System.out.println("Operation PANEL CONFIGURED!!! Operation PANEL CONFIGURED!!!");
  }

  private void setupDriveController() {
  }

  public void testPeriodic() {
  }
}

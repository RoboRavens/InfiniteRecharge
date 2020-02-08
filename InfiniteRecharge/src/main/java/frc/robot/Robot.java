/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.controls.Gamepad;
import frc.controls.OperationPanel;
import frc.robot.commands.drivetrain.DriveTrainStopCommand;
import frc.robot.commands.shooter.ControlPanelShotCommand;
import frc.robot.commands.shooter.InitiationLineShotCommand;
import frc.robot.commands.shooter.ShooterRevCommand;
import frc.robot.commands.shooter.ShooterRumbleFeedbackCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
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
  
  public ShooterRevCommand shooterRev = new ShooterRevCommand(Robot.SHOOTER_SUBSYSTEM);
  public ControlPanelShotCommand shooterCtrlPanelShot = new ControlPanelShotCommand(Robot.SHOOTER_SUBSYSTEM);
  public InitiationLineShotCommand shooterLineShot = new InitiationLineShotCommand(Robot.SHOOTER_SUBSYSTEM);
  public ShooterTuneCommand shooterTune = new ShooterTuneCommand(Robot.SHOOTER_SUBSYSTEM);

  public ShooterRumbleFeedbackCommand shooterRumble = new ShooterRumbleFeedbackCommand(Robot.SHOOTER_SUBSYSTEM);

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    CLIMBER_SUBSYSTEM.resetEncodersToRetractedLimit();
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.resetDriveEncoders();
    INTAKE_SUBSYSTEM.intakeRetract();
    LIMELIGHT_SUBSYSTEM.turnLEDOff();
    this.setupDefaultCommands();
    this.setupShooterController();
    this.setupDriveController();
    this.setupOperationPanel();
  }


  private void setupDefaultCommands() {
    DRIVE_TRAIN_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> DRIVE_TRAIN_SUBSYSTEM.defaultCommand(), DRIVE_TRAIN_SUBSYSTEM));
    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  public void teleopPeriodic() {
  }

  public void setupShooterController() {
    System.out.println("DRIVE CONTROLLER CONFIGURED");
    System.out.println("Remember to enable to actually make things work");
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.A).whileHeld(shooterRev);
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.B).whileHeld(shooterTune);
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.X).whenPressed(shooterLineShot);
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

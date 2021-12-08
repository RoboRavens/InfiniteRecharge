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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.controls.Gamepad;
import frc.controls.OperationPanel;
import frc.controls.OperationPanel2;
import frc.ravenhardware.RavenBlinkin;
import frc.robot.commands.autonomous.DriveAndShootAutonomousCommand;
import frc.robot.commands.autonomous.DriveAutonomousCommand;
import frc.robot.commands.autonomous.NamedAutonomousCommand;
import frc.robot.commands.autonomous.SixBallCenteredAutonomousCommand;
import frc.robot.commands.autonomous.SixBallSideAutonomousCommand;
import frc.robot.commands.climber.ClimberExtendFullyCommand;
import frc.robot.commands.climber.ClimberExtendWhileHeldCommand;
import frc.robot.commands.climber.ClimberExtendWithEncoderCommand;
import frc.robot.commands.climber.ClimberRetractFullyCommand;
import frc.robot.commands.climber.ClimberRetractWhileHeldCommand;
import frc.robot.commands.climber.ClimberRetractWithEncoderCommand;
import frc.robot.commands.climber.pidclimb.ClimberPIDExtendCommand;
import frc.robot.commands.climber.pidclimb.ClimberPIDRetractCommand;
import frc.robot.commands.conveyance.ConveyanceReverseCommand;
import frc.robot.commands.conveyance.ConveyanceReverseForDurationCommand;
import frc.robot.commands.conveyance.ConveyanceShootWhileHeldCommand;
import frc.robot.commands.conveyance.ConveyanceSlowFeedCommand;
import frc.robot.commands.drivetrain.DriveTrainTurnToTargetCommand;
import frc.robot.commands.hopper.HopperAgitateCommand;
import frc.robot.commands.intake.IntakeExtendAndCollectCommand;
import frc.robot.commands.powercells.ReadyToShootCommandGroup;
import frc.robot.commands.powercells.RevDownCommandGroup;
import frc.robot.commands.powercells.ShooterShootWhenReadyCommand;
import frc.robot.commands.powercells.StopConveyanceCommandGroup;
import frc.robot.commands.shooter.SetShotCloseTrenchCommand;
import frc.robot.commands.shooter.SetShotFarTrenchCommand;
import frc.robot.commands.shooter.SetShotInitCommand;
import frc.robot.commands.shooter.ShooterRevCommand;
import frc.robot.commands.utility.CompressorTurnOffWhileShootingCommand;
import frc.robot.commands.utility.SleepCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.util.LoggerOverlord;
import frc.util.OverrideSystem;

public class Robot extends TimedRobot {
  private final SendableChooser<NamedAutonomousCommand> autonomousChooser = new SendableChooser<>();
  private final SendableChooser<Integer> autonomousDelayChooser = new SendableChooser<>();

  public DriverStation driverStation;
  public PowerDistributionPanel PDP = new PowerDistributionPanel();
  // private RavenBlinkin ravenBlinkin;

  public static final LoggerOverlord LOGGER_OVERLORD = new LoggerOverlord(1f);
  public static final Gamepad DRIVE_CONTROLLER = new Gamepad(0);
  public static final OperationPanel OPERATION_PANEL = new OperationPanel(1);
  public static final OperationPanel2 OPERATION_PANEL_2 = new OperationPanel2(2);

  public static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();
  public static final CompressorSubsystem COMPRESSOR_SUBSYSTEM = new CompressorSubsystem();
  public static final ConveyanceSubsystem CONVEYANCE_SUBSYSTEM = new ConveyanceSubsystem();
  public static final DriveTrainSubsystem DRIVE_TRAIN_SUBSYSTEM = new DriveTrainSubsystem();
  public static final HopperSubsystem HOPPER_SUBSYSTEM = new HopperSubsystem();
  public static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
  public static final TurretSubsystem TURRET_SUBSYSTEM = new TurretSubsystem();

  public static final OverrideSystem OVERRIDE_SYSTEM_CLIMBER_EXTEND = new OverrideSystem();
  public static final OverrideSystem OVERRIDE_SYSTEM_CLIMBER_RETRACT = new OverrideSystem();

  public static boolean isRedAlliance;

  public String autoFromDashboard;
  public String positionFromDashboard;

  // COMMANDS
  public ShooterRevCommand shooterRev = new ShooterRevCommand();
  public ReadyToShootCommandGroup readyToShoot = new ReadyToShootCommandGroup();
  public StopConveyanceCommandGroup stopConveyance = new StopConveyanceCommandGroup();
  public RevDownCommandGroup revDown = new RevDownCommandGroup();
  public ConveyanceReverseCommand conveyanceReverse = new ConveyanceReverseCommand();
  public HopperAgitateCommand hopperAgitate = new HopperAgitateCommand();
  public ClimberRetractWhileHeldCommand climberRetract = new ClimberRetractWhileHeldCommand();
  public ClimberRetractWithEncoderCommand climberRetractFully = new ClimberRetractWithEncoderCommand();
  public ClimberExtendWhileHeldCommand climberExtend = new ClimberExtendWhileHeldCommand();
  public ClimberExtendWithEncoderCommand climberExtendFully = new ClimberExtendWithEncoderCommand();
  public ClimberPIDExtendCommand climberPIDExtend = new ClimberPIDExtendCommand();
  public ClimberPIDRetractCommand climberPIDRetract = new ClimberPIDRetractCommand();
  public IntakeExtendAndCollectCommand intakeAndCollect = new IntakeExtendAndCollectCommand();
  public ConveyanceSlowFeedCommand conveyanceSlowFeed = new ConveyanceSlowFeedCommand();
  public SetShotInitCommand setShotInit = new SetShotInitCommand();
  public SetShotCloseTrenchCommand setShotCloseTrench = new SetShotCloseTrenchCommand();
  public SetShotFarTrenchCommand setShotFarTrench = new SetShotFarTrenchCommand();
  public ShooterShootWhenReadyCommand shootWhenReady = new ShooterShootWhenReadyCommand();

  public ConveyanceShootWhileHeldCommand conveyanceShootWhileHeld = new ConveyanceShootWhileHeldCommand();

  public DriveTrainTurnToTargetCommand TurnToTarget = new DriveTrainTurnToTargetCommand();

  public CompressorTurnOffWhileShootingCommand compressorOffWhenShooting = new CompressorTurnOffWhileShootingCommand();

  public String currentAutoName = "";

  @Override
  public void robotInit() {
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.resetOdometry();
    INTAKE_SUBSYSTEM.retract();
    LIMELIGHT_SUBSYSTEM.turnLEDOff();
    this.setupDefaultCommands();
    this.setupDriveController();
    this.setupOperationPanel();
    this.setupAutonomousCommands();
  }

  private void setupDefaultCommands() {
    CLIMBER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> CLIMBER_SUBSYSTEM.defaultCommand(), CLIMBER_SUBSYSTEM));
    CONVEYANCE_SUBSYSTEM
        .setDefaultCommand(new RunCommand(() -> CONVEYANCE_SUBSYSTEM.defaultCommand(), CONVEYANCE_SUBSYSTEM));
    DRIVE_TRAIN_SUBSYSTEM
        .setDefaultCommand(new RunCommand(() -> DRIVE_TRAIN_SUBSYSTEM.defaultCommand(), DRIVE_TRAIN_SUBSYSTEM));
    HOPPER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> HOPPER_SUBSYSTEM.defaultCommand(), HOPPER_SUBSYSTEM));
    INTAKE_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> INTAKE_SUBSYSTEM.defaultCommand(), INTAKE_SUBSYSTEM));
    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));
    TURRET_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> TURRET_SUBSYSTEM.defaultCommand(), TURRET_SUBSYSTEM));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.tankDriveVolts(0, 0);
    Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
  }

  @Override
  public void autonomousInit() {
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.resetOdometry();
    Command autonomousCommand = autonomousChooser.getSelected().Command;
    SequentialCommandGroup autonomousWithDelayCommand = new SequentialCommandGroup(
        new SleepCommand("delay autonomous", autonomousDelayChooser.getSelected()), autonomousCommand);

    if (autonomousCommand != null) {
      autonomousWithDelayCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.logPose();
  }

  @Override
  public void teleopInit() {
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.setGyroTargetHeadingToCurrentHeading();
  }

  public void teleopPeriodic() {
    // System.out.print("Angle: " + LIMELIGHT_SUBSYSTEM.isAlignedToTarget());
    // System.out.print(" Button: " +
    // DRIVE_CONTROLLER.getButtonValue(ButtonCode.LEFTBUMPER));
    // System.out.print(" RPM: " +
    // SHOOTER_SUBSYSTEM.getIsInInitiationLineRpmRange());
    // System.out.print(" Override Off: " +
    // OPERATION_PANEL.getButtonValue(ButtonCode.SHOOTING_MODE_OVERRIDE));
    // System.out.println(" RPM: " + SHOOTER_SUBSYSTEM.getRPM());
    // System.out.println(" RTS: " + SHOOTER_SUBSYSTEM.readyToShoot());

    if (OPERATION_PANEL_2.getButton(ButtonCode.SETSHOTFARCONTROLPANEL).get()) {
      System.out.println("CLIMBER OVERRIDE ACTIVATED");
      CLIMBER_SUBSYSTEM.setOverrideOn();
    }
    else {
      CLIMBER_SUBSYSTEM.setOverrideOff();
    }
    
    // DRIVE_TRAIN_SUBSYSTEM.ravenTank.logPose();
    Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
    if (DRIVE_TRAIN_SUBSYSTEM.ravenTank.userControlOfCutPower) {
      if (DRIVE_CONTROLLER.getAxis(AxisCode.RIGHTTRIGGER) > .25
          || DRIVE_CONTROLLER.getButtonValue(ButtonCode.RIGHTBUMPER)) {
        System.out.println("CUT POWER TRUE");
        DRIVE_TRAIN_SUBSYSTEM.ravenTank.setCutPower(true);
      } else {
        DRIVE_TRAIN_SUBSYSTEM.ravenTank.setCutPower(false);
      }
    }
    if (DRIVE_CONTROLLER.getAxis(AxisCode.LEFTTRIGGER) > .25) {
      Robot.LIMELIGHT_SUBSYSTEM.turnLEDOn();
      // System.out.println("TURNING TO TARGET");
      TurnToTarget.schedule();
    }
    
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.LEFTBUMPER).whileHeld(shootWhenReady);
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.RIGHTBUMPER).whileHeld(intakeAndCollect);
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.RIGHTBUMPER).whileHeld(conveyanceSlowFeed);

    /*
    if (Timer.getMatchTime() == 60) {
      ravenBlinkin.flashGreen();
    } else if (Timer.getMatchTime() == 30) {
      ravenBlinkin.flashYellow();
    } else if (Timer.getMatchTime() == 15) {
      ravenBlinkin.flashRed();
    }
    */
    
  }

  public void setupOperationPanel() {
    // System.out.println("Operation PANEL CONFIGURED!!! Operation PANEL CONFIGURED!!!");

    Robot.OPERATION_PANEL.getButton(ButtonCode.READYTOSHOOT).whileHeld(readyToShoot);
    Robot.OPERATION_PANEL.getButton(ButtonCode.SHOOTERREV)
        .whileHeld(new SequentialCommandGroup(new SleepCommand("delay rev", .15), shooterRev));
    Robot.OPERATION_PANEL.getButton(ButtonCode.SHOOTERREV).whenPressed(new ConveyanceReverseForDurationCommand(.15));
    Robot.OPERATION_PANEL.getButton(ButtonCode.SHOOTERREV).whenReleased(revDown);
    Robot.OPERATION_PANEL.getButton(ButtonCode.OVERRIDEREVERSECONVEYANCE).whileHeld(conveyanceReverse);
    Robot.OPERATION_PANEL.getButton(ButtonCode.OVERRIDECLIMBEXTEND).whileHeld(climberExtend);
    //Robot.OPERATION_PANEL.getButton(ButtonCode.SETCLIMBERTOPOSITION).whenPressed(climberExtendFully);
    // ^ May want to make a command group that retracts to latch after extending
    // fully ^
    Robot.OPERATION_PANEL.getButton(ButtonCode.OVERRIDECLIMBRETRACT).whileHeld(climberRetract);
    //Robot.OPERATION_PANEL.getButton(ButtonCode.SETCLIMBERTORETRACTED).whenPressed(climberRetractFully);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.HOPPERAGITATE).whileHeld(hopperAgitate);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.CONVEYANCESHOOT).whileHeld(conveyanceShootWhileHeld);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.CONVEYANCESHOOT).whileHeld(hopperAgitate);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.SETSHOTINITIATIONLINE).whenPressed(setShotInit);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.SETSHOTCLOSECONTROLPANEL).whenPressed(setShotCloseTrench);
    // Robot.OPERATION_PANEL_2.getButton(ButtonCode.SETSHOTFARCONTROLPANEL).whenPressed(setShotFarTrench);

    Robot.OPERATION_PANEL.getButton(ButtonCode.SETCLIMBERTOPOSITION).whenPressed(climberPIDExtend);
    Robot.OPERATION_PANEL.getButton(ButtonCode.SETCLIMBERTORETRACTED).whenPressed(climberPIDRetract);
  }

  private void setupDriveController() {
    System.out.println("DRIVE CONTROLLER CONFIGURED");
  }

  public void testPeriodic() {
  }

  public void disabledPeriodic() {
    Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
    SmartDashboard.putString("DB/String 0", autonomousChooser.getSelected().Name);
    SmartDashboard.putString("Autonomous Mode", autonomousChooser.getSelected().Name);

    SmartDashboard.putNumber("DB/String 1", autonomousDelayChooser.getSelected());
    SmartDashboard.putNumber("Autonomous Delay", autonomousDelayChooser.getSelected());
  }

  private void setupAutonomousCommands() {
    autonomousChooser.setDefaultOption("Drive and Shoot",
        new NamedAutonomousCommand("Drive and Shoot", DriveAndShootAutonomousCommand.GenerateCommand()));
    autonomousChooser.addOption("Six Ball Centered",
        new NamedAutonomousCommand("Six Ball Centered", SixBallCenteredAutonomousCommand.GenerateCommand()));
    autonomousChooser.addOption("Six Ball Side",
        new NamedAutonomousCommand("Six Ball Side", SixBallSideAutonomousCommand.GenerateCommand()));
    autonomousChooser.addOption("Do Nothing", new NamedAutonomousCommand("Do Nothing", new InstantCommand()));
    autonomousChooser.addOption("Drive", new NamedAutonomousCommand("Drive", DriveAutonomousCommand.GenerateCommand()));

    autonomousDelayChooser.setDefaultOption("No Delay", 0);
    autonomousDelayChooser.addOption("1 Second", 1);
    autonomousDelayChooser.addOption("2 Seconds", 2);
    autonomousDelayChooser.addOption("3 Seconds", 3);
    autonomousDelayChooser.addOption("4 Seconds", 4);
    autonomousDelayChooser.addOption("5 Seconds", 5);

    SmartDashboard.putData("Autonomous Choices", autonomousChooser);
    SmartDashboard.putData("Autonomous Delay", autonomousDelayChooser);
  }
}

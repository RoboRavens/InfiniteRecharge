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
import frc.robot.commands.climber.ClimberRetractFullyCommand;
import frc.robot.commands.climber.ClimberRetractWhileHeldCommand;
import frc.robot.commands.conveyance.ConveyanceReverseCommand;
import frc.robot.commands.conveyance.ConveyanceReverseForDurationCommand;
import frc.robot.commands.conveyance.ConveyanceShootWhileHeldCommand;
import frc.robot.commands.conveyance.ConveyanceSlowFeedCommand;
import frc.robot.commands.drivetrain.DriveTrainTurnTargetCommand;
import frc.robot.commands.hopper.HopperAgitateCommand;
import frc.robot.commands.intake.IntakeExtendAndCollectCommand;
import frc.robot.commands.powercells.ReadyToShootCommandGroup;
import frc.robot.commands.powercells.RevDownCommandGroup;
import frc.robot.commands.powercells.StopConveyanceCommandGroup;
import frc.robot.commands.shooter.SetShotCloseTrenchCommand;
import frc.robot.commands.shooter.SetShotFarTrenchCommand;
import frc.robot.commands.shooter.SetShotInitCommand;
import frc.robot.commands.shooter.ShooterRevCommand;
import frc.robot.commands.utility.SleepCommand;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.ConveyanceSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.util.LoggerOverlord;
import frc.util.OverrideSystem;

public class Robot extends TimedRobot {
  private final SendableChooser<NamedAutonomousCommand> autonomousChooser = new SendableChooser<>();

  public DriverStation driverStation;
  public PowerDistributionPanel PDP = new PowerDistributionPanel();
  public RavenBlinkin timeBlinkin = new RavenBlinkin(1);

  public static final LoggerOverlord LOGGER_OVERLORD = new LoggerOverlord(1f);
  public static final Gamepad DRIVE_CONTROLLER = new Gamepad(0);
  public static final OperationPanel OPERATION_PANEL = new OperationPanel(1);
  public static final OperationPanel2 OPERATION_PANEL_2 = new OperationPanel2(2);
  
  public static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();
  //public static final CompressorSubsystem COMPRESSOR_SUBSYSTEM = new CompressorSubsystem();
  public static final ConveyanceSubsystem CONVEYANCE_SUBSYSTEM = new ConveyanceSubsystem();
  public static final DriveTrainSubsystem DRIVE_TRAIN_SUBSYSTEM = new DriveTrainSubsystem();
  public static final HopperSubsystem HOPPER_SUBSYSTEM = new HopperSubsystem();
  public static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  public static final LimelightSubsystem LIMELIGHT_SUBSYSTEM = new LimelightSubsystem();
  public static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();

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
  public ClimberRetractFullyCommand climberRetractFully = new ClimberRetractFullyCommand();
  public ClimberExtendWhileHeldCommand climberExtend = new ClimberExtendWhileHeldCommand();
  public ClimberExtendFullyCommand climberExtendFully = new ClimberExtendFullyCommand();
  public IntakeExtendAndCollectCommand intakeAndCollect = new IntakeExtendAndCollectCommand();
  public ConveyanceSlowFeedCommand conveyanceSlowFeed = new ConveyanceSlowFeedCommand();
  public SetShotInitCommand setShotInit = new SetShotInitCommand();
  public SetShotCloseTrenchCommand setShotCloseTrench = new SetShotCloseTrenchCommand();
  public SetShotFarTrenchCommand setShotFarTrench = new SetShotFarTrenchCommand();

  public ConveyanceShootWhileHeldCommand conveyanceShootWhileHeld = new ConveyanceShootWhileHeldCommand();

  public DriveTrainTurnTargetCommand turnTarget = new DriveTrainTurnTargetCommand();

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
    CONVEYANCE_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> CONVEYANCE_SUBSYSTEM.defaultCommand(), CONVEYANCE_SUBSYSTEM));
    DRIVE_TRAIN_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> DRIVE_TRAIN_SUBSYSTEM.defaultCommand(), DRIVE_TRAIN_SUBSYSTEM));
    HOPPER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> HOPPER_SUBSYSTEM.defaultCommand(), HOPPER_SUBSYSTEM));
    INTAKE_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> INTAKE_SUBSYSTEM.defaultCommand(), INTAKE_SUBSYSTEM));
    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM));
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

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
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
    //System.out.print("Angle: " + LIMELIGHT_SUBSYSTEM.isAlignedToTarget());
    //System.out.print(" Button: " + DRIVE_CONTROLLER.getButtonValue(ButtonCode.LEFTBUMPER));
    //System.out.print(" RPM: " + SHOOTER_SUBSYSTEM.getIsInInitiationLineRpmRange());
    //System.out.print(" Override Off: " + OPERATION_PANEL.getButtonValue(ButtonCode.SHOOTING_MODE_OVERRIDE));
    //System.out.println(" RPM: " + SHOOTER_SUBSYSTEM.getRPM());
    //System.out.println(" RTS: " + SHOOTER_SUBSYSTEM.readyToShoot());

    // DRIVE_TRAIN_SUBSYSTEM.ravenTank.logPose();
    Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
    if (DRIVE_TRAIN_SUBSYSTEM.ravenTank.userControlOfCutPower) {
			if (DRIVE_CONTROLLER.getAxis(AxisCode.RIGHTTRIGGER) > .25 || DRIVE_CONTROLLER.getButtonValue(ButtonCode.RIGHTBUMPER)) {
				System.out.println("CUT POWER TRUE");
			  DRIVE_TRAIN_SUBSYSTEM.ravenTank.setCutPower(true);
			}
			else {
			  DRIVE_TRAIN_SUBSYSTEM.ravenTank.setCutPower(false);
			}
    }
    if (DRIVE_CONTROLLER.getAxis(AxisCode.LEFTTRIGGER) > .25) {
      Robot.LIMELIGHT_SUBSYSTEM.turnLEDOn();
      // System.out.println("TURNING TO TARGET");
      turnTarget.schedule();
    }
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.RIGHTBUMPER).whileHeld(intakeAndCollect);
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.RIGHTBUMPER).whileHeld(conveyanceSlowFeed);

    timeBlinkin.checkTime(Timer.getMatchTime());
  }


  public void setupOperationPanel() {
    System.out.println("Operation PANEL CONFIGURED!!! Operation PANEL CONFIGURED!!!");
    
    Robot.OPERATION_PANEL.getButton(ButtonCode.READYTOSHOOT).whileHeld(readyToShoot);
    Robot.OPERATION_PANEL.getButton(ButtonCode.SHOOTERREV).whileHeld(
      new SequentialCommandGroup(
        new SleepCommand("delay rev", .15),
        shooterRev));
    Robot.OPERATION_PANEL.getButton(ButtonCode.SHOOTERREV).whenPressed(new ConveyanceReverseForDurationCommand(.15));
    Robot.OPERATION_PANEL.getButton(ButtonCode.SHOOTERREV).whenReleased(revDown);
    Robot.OPERATION_PANEL.getButton(ButtonCode.OVERRIDEREVERSECONVEYANCE).whileHeld(conveyanceReverse);
    Robot.OPERATION_PANEL.getButton(ButtonCode.OVERRIDECLIMBEXTEND).whileHeld(climberExtend);
    Robot.OPERATION_PANEL.getButton(ButtonCode.SETCLIMBERTOPOSITION).whenPressed(climberExtendFully);
    // ^ May want to make a command group that retracts to latch after extending fully ^
    Robot.OPERATION_PANEL.getButton(ButtonCode.OVERRIDECLIMBRETRACT).whileHeld(climberRetract);
    Robot.OPERATION_PANEL.getButton(ButtonCode.SETCLIMBERTORETRACTED).whenPressed(climberRetractFully);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.HOPPERAGITATE).whileHeld(hopperAgitate);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.CONVEYANCESHOOT).whileHeld(conveyanceShootWhileHeld);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.CONVEYANCESHOOT).whileHeld(hopperAgitate);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.SETSHOTINITIATIONLINE).whenPressed(setShotInit);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.SETSHOTCLOSECONTROLPANEL).whenPressed(setShotCloseTrench);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.SETSHOTFARCONTROLPANEL).whenPressed(setShotFarTrench);
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

    // if (autonomousChooser.getSelected().Name.equals(this.currentAutoName))

    // System.out.println("Net Inches Traveled: " + DRIVE_TRAIN_SUBSYSTEM.ravenTank.getRightNetInchesTraveled());
  }

  private void setupAutonomousCommands() {
    autonomousChooser.setDefaultOption("Do Nothing", new NamedAutonomousCommand("Do Nothing", new InstantCommand()));
    autonomousChooser.addOption("Six Ball Centered", new NamedAutonomousCommand("Six Ball Centered", SixBallCenteredAutonomousCommand.GenerateCommand()));
    autonomousChooser.addOption("Six Ball Side", new NamedAutonomousCommand("Six Ball Side", SixBallSideAutonomousCommand.GenerateCommand()));
    autonomousChooser.addOption("Drive and Shoot", new NamedAutonomousCommand("Drive and Shoot", DriveAndShootAutonomousCommand.GenerateCommand()));
    autonomousChooser.addOption("Drive", new NamedAutonomousCommand("Drive", DriveAutonomousCommand.GenerateCommand()));
    
    SmartDashboard.putData("Autonomous Choices", autonomousChooser);
  }
}

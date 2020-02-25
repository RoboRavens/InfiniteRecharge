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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.controls.AxisCode;
import frc.controls.ButtonCode;
import frc.controls.Gamepad;
import frc.controls.OperationPanel;
import frc.controls.OperationPanel2;
import frc.robot.commands.climber.ClimberExtendFullyCommand;
import frc.robot.commands.climber.ClimberExtendWhileHeldCommand;
import frc.robot.commands.climber.ClimberRetractFullyCommand;
import frc.robot.commands.climber.ClimberRetractWhileHeldCommand;
import frc.robot.commands.conveyance.ConveyanceReverseCommand;
import frc.robot.commands.drivetrain.DriveTrainTurnTargetCommand;
import frc.robot.commands.hopper.HopperAgitateCommand;
import frc.robot.commands.intake.IntakeExtendAndCollectCommand;
import frc.robot.commands.intake.IntakeRetractCommand;
import frc.robot.commands.powercells.IntakeToReadyCommandGroup;
import frc.robot.commands.powercells.ReadyToShootCommandGroup;
import frc.robot.commands.powercells.RevDownCommandGroup;
import frc.robot.commands.powercells.RunShooterCommandGroup;
import frc.robot.commands.powercells.ShootIfReadyCommandGroup;
import frc.robot.commands.powercells.StopConveyanceCommandGroup;
import frc.robot.commands.shooter.SetShotControlPanelCommand;
import frc.robot.commands.shooter.SetShotInitiationLineCommand;
import frc.robot.commands.shooter.ShooterRevCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
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
  public static final OperationPanel2 OPERATION_PANEL_2 = new OperationPanel2(2);
  
  public static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();
  public static final CompressorSubsystem COMPRESSOR_SUBSYSTEM = new CompressorSubsystem();
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

  // COMMANDS
  public ShooterRevCommand shooterRev = new ShooterRevCommand(10000); //Robot.SHOOTER_SUBSYSTEM.getTargetRPM()
  public ReadyToShootCommandGroup readyToShoot = new ReadyToShootCommandGroup();
  public SetShotControlPanelCommand setShotControlPanel = new SetShotControlPanelCommand();
  public SetShotInitiationLineCommand setShotInitiationLine = new SetShotInitiationLineCommand();
  public IntakeToReadyCommandGroup intakeToReady = new IntakeToReadyCommandGroup();
  public StopConveyanceCommandGroup stopConveyance = new StopConveyanceCommandGroup();
  public RunShooterCommandGroup runShooter = new RunShooterCommandGroup();
  public RevDownCommandGroup revDown = new RevDownCommandGroup();
  public ConveyanceReverseCommand conveyanceReverse = new ConveyanceReverseCommand();
  public HopperAgitateCommand hopperAgitate = new HopperAgitateCommand();
  public ClimberRetractWhileHeldCommand climberRetract = new ClimberRetractWhileHeldCommand();
  public ClimberRetractFullyCommand climberRetractFully = new ClimberRetractFullyCommand();
  public ClimberExtendWhileHeldCommand climberExtend = new ClimberExtendWhileHeldCommand();
  public ClimberExtendFullyCommand climberExtendFully = new ClimberExtendFullyCommand();
  public IntakeExtendAndCollectCommand intakeAndCollect = new IntakeExtendAndCollectCommand();
  public ShootIfReadyCommandGroup shootIfReady = new ShootIfReadyCommandGroup();

  public DriveTrainTurnTargetCommand turnTarget = new DriveTrainTurnTargetCommand();

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    DRIVE_TRAIN_SUBSYSTEM.ravenTank.resetOdometry();
    INTAKE_SUBSYSTEM.retract();
    LIMELIGHT_SUBSYSTEM.turnLEDOff();
    this.setupDefaultCommands();
    this.setupDriveController();
    this.setupOperationPanel();
  }

  private void setupDefaultCommands() {
    /* CLIMBER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> CLIMBER_SUBSYSTEM.defaultCommand(), CLIMBER_SUBSYSTEM));
    CONVEYANCE_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> CONVEYANCE_SUBSYSTEM.defaultCommand(), CONVEYANCE_SUBSYSTEM));
    DRIVE_TRAIN_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> DRIVE_TRAIN_SUBSYSTEM.defaultCommand(), DRIVE_TRAIN_SUBSYSTEM));
    HOPPER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> HOPPER_SUBSYSTEM.defaultCommand(), HOPPER_SUBSYSTEM));
    INTAKE_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> INTAKE_SUBSYSTEM.defaultCommand(), INTAKE_SUBSYSTEM));
    SHOOTER_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> SHOOTER_SUBSYSTEM.defaultCommand(), SHOOTER_SUBSYSTEM)); */

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

  private Command GetReverseTrajectoryTest(){
    // An example trajectory to follow.  All units in meters.
    var forwardTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      // List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
      List.of(
        new Translation2d(1, 0),
        new Translation2d(2, 1),
        new Translation2d(3, 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(4, 0, new Rotation2d(0)),
      // Pass config
      DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig()
    );

    var reverseTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(4, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      // List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
      List.of(
        new Translation2d(3, 0),
        new Translation2d(2, 1),
        new Translation2d(1, 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass config
      DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig().setReversed(true)
    );

    var autonomousCommand1 = DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(forwardTrajectory);
    var autonomousCommand2 = DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(reverseTrajectory);

    return new SequentialCommandGroup(autonomousCommand1, autonomousCommand2, new InstantCommand(() -> DRIVE_TRAIN_SUBSYSTEM.ravenTank.setGyroTargetHeadingToCurrentHeading(), DRIVE_TRAIN_SUBSYSTEM), new InstantCommand(()-> System.out.println("Drive Command Finished!")));
  }

  public Trajectory GetPathweaverTrajectoryForAuto(){
    try {
      SmartDashboard.putString("PathFollowed", m_chooser.getSelected());
      var trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/" + m_chooser.getSelected() + ".wpilib.json"));
      return DRIVE_TRAIN_SUBSYSTEM.ravenTank.reverseTrajectory(trajectory);
    } catch (IOException e) {
      e.printStackTrace();
    }

    return null;
  }

  public Command GetReversePathweaverTrajectoryTest(){
    try {
      SmartDashboard.putString("PathFollowed", m_chooser.getSelected());
      var trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/Line.wpilib.json"));
      var trajectoryToReverse = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/ReverseLine.wpilib.json"));
      var reverseTrajectory = DRIVE_TRAIN_SUBSYSTEM.ravenTank.reverseTrajectory2(trajectoryToReverse);
      var trajectoryCommand = DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(trajectory);
      var reverseTrajectoryCommand = DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(reverseTrajectory);
      return new SequentialCommandGroup(trajectoryCommand, reverseTrajectoryCommand);
    } catch (IOException e) {
      e.printStackTrace();
    }

    return null;
  }

  public Command GetAutonomousCommand(){
    var trajectory1 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(3.558, -2.404, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      // List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
      List.of(
        new Translation2d(4.219, -1.542)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(5.2, -0.705, new Rotation2d(0)),
      // Pass config
      DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig()
    );

    var trajectory2 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(5.2, -0.705, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      // List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
      List.of(),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(7.588, -0.705, new Rotation2d(0)),
      // Pass config
      DRIVE_TRAIN_SUBSYSTEM.ravenTank.getTrajectoryConfig()
    );

    DRIVE_TRAIN_SUBSYSTEM.ravenTank.setOdemetry(trajectory1.getInitialPose());

    var autonomousCommand1 = DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(trajectory1);
    var autonomousCommand2 = DRIVE_TRAIN_SUBSYSTEM.ravenTank.getCommandForTrajectory(trajectory2);

    return new SequentialCommandGroup(autonomousCommand1,
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          autonomousCommand2,
          new InstantCommand(() -> DRIVE_TRAIN_SUBSYSTEM.ravenTank.setGyroTargetHeadingToCurrentHeading(), DRIVE_TRAIN_SUBSYSTEM),
          new IntakeRetractCommand()
        ),
        new IntakeExtendAndCollectCommand()
      ),
      new InstantCommand(()-> System.out.println("Drive Command Finished!")));
  }

  @Override
  public void autonomousInit() {
    DRIVE_TRAIN_SUBSYSTEM.ravenTank.resetOdometry();
    Command autonomousCommand = GetAutonomousCommand();

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
    // DRIVE_TRAIN_SUBSYSTEM.ravenTank.logPose();
    Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
    if (DRIVE_TRAIN_SUBSYSTEM.ravenTank.userControlOfCutPower) {
			if (DRIVE_CONTROLLER.getAxis(AxisCode.RIGHTTRIGGER) > .25) {
				System.out.println("CUT POWER TRUE");
			  DRIVE_TRAIN_SUBSYSTEM.ravenTank.setCutPower(true);
			}
			else {
			  DRIVE_TRAIN_SUBSYSTEM.ravenTank.setCutPower(false);
			}
    }
    if (DRIVE_CONTROLLER.getAxis(AxisCode.LEFTTRIGGER) > .25) {
      Robot.LIMELIGHT_SUBSYSTEM.turnLEDOn();
      System.out.println("TURNING TO TARGET");
      turnTarget.schedule();
    }

    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.LEFTBUMPER).whileHeld(shootIfReady);
    Robot.DRIVE_CONTROLLER.getButton(ButtonCode.RIGHTBUMPER).whileHeld(intakeAndCollect);
  }

  public void setupOperationPanel() {
    System.out.println("Operation PANEL CONFIGURED!!! Operation PANEL CONFIGURED!!!");
    
    Robot.OPERATION_PANEL.getButton(ButtonCode.READYTOSHOOT).whileHeld(readyToShoot);
    Robot.OPERATION_PANEL.getButton(ButtonCode.SHOOTERREV).whileHeld(shooterRev);
    Robot.OPERATION_PANEL.getButton(ButtonCode.SHOOTERREV).whenReleased(revDown);
    Robot.OPERATION_PANEL.getButton(ButtonCode.OVERRIDEREVERSECONVEYANCE).whileHeld(conveyanceReverse);
    Robot.OPERATION_PANEL.getButton(ButtonCode.OVERRIDECLIMBEXTEND).whileHeld(climberExtend);
    Robot.OPERATION_PANEL.getButton(ButtonCode.SETCLIMBERTOPOSITION).whenPressed(climberExtendFully);
    // ^ May want to make a command group that retracts to latch after extending fully ^
    Robot.OPERATION_PANEL.getButton(ButtonCode.OVERRIDECLIMBRETRACT).whileHeld(climberRetract);
    Robot.OPERATION_PANEL.getButton(ButtonCode.SETCLIMBERTORETRACTED).whenPressed(climberRetractFully);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.SETSHOTCONTROLPANEL).whenPressed(setShotControlPanel);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.SETSHOTINITIATIONLINE).whenPressed(setShotInitiationLine);
    Robot.OPERATION_PANEL_2.getButton(ButtonCode.HOPPERAGITATE).whileHeld(hopperAgitate);
  }

  private void setupDriveController() {
    System.out.println("DRIVE CONTROLLER CONFIGURED");
  }

  public void testPeriodic() {
  }

  public void disabledPeriodic() {
    Robot.LIMELIGHT_SUBSYSTEM.turnLEDOff();
  }
}

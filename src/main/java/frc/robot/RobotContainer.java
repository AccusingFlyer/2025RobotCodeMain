// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.xboxDriver.DRIVE_CONTROLLER_ID;
import static frc.robot.settings.Constants.xboxDriver.OPERATOR_CONTROLLER_ID;
import static frc.robot.settings.Constants.xboxDriver.X_AXIS;
import static frc.robot.settings.Constants.xboxDriver.Y_AXIS;
import static frc.robot.settings.Constants.xboxDriver.Z_AXIS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive.Drive;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Elevator.ElevatorManualCommand;
import frc.robot.commands.EndEffector.ClawRoller;
import frc.robot.commands.EndEffector.WristSetpointCommand;
// import frc.robot.commands.IntakePowerCommand;
// import frc.robot.commands.IntakeSetpointCommand;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Vision;
import frc.robot.subsystems.AutoReefAlignSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotState;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // preferences are information saved on the Rio. They are initialized once, then
  // gotten every time
  // we run the code.

  private final boolean DrivetrainExists = Preferences.getBoolean("DrivetrainExists", true);

  private DrivetrainSubsystem driveTrain;
  private ElevatorSubsystem elevator;
  // private IntakeSubsystem intake;
  private EndEffectorSubsystem endEffector;
  private AutoReefAlignSubsystem reefAlign;

  private Drive defaultDriveCommand;

  private XboxController driverControllerXbox;
  public static XboxController operatorControllerXbox;

  private Limelight limelight;
  private Lights lights;
  private SendableChooser<Command> autoChooser;
  private PowerDistribution PDP;

  RobotState robotState;
  Alliance currentAlliance;
  BooleanSupplier ZeroGyroSup;
  BooleanSupplier LeftReefLineupSup;
  BooleanSupplier RightReefLineupSup;
  BooleanSupplier SlowFrontSup;
  BooleanSupplier AlgaeIntakeSup;
  BooleanSupplier CoralIntakeSup;
  BooleanSupplier AlgaeProcessorPositionSup;
  BooleanSupplier CoralReefScoreSup;
  BooleanSupplier ReefHeight1Supplier;
  BooleanSupplier ReefHeight2Supplier;
  BooleanSupplier ReefHeight3Supplier;
  BooleanSupplier ReefHeight4Supplier;
  BooleanSupplier BargeHeightSupplier;
  BooleanSupplier CoralIntakeHeightSupplier;
  BooleanSupplier removeAlgaeHighSupplier;
  BooleanSupplier removeAlgaeLowSupplier;
  // BooleanSupplier ReefA;
  // BooleanSupplier ReefB;
  // BooleanSupplier ReefC;
  // BooleanSupplier ReefD;
  // BooleanSupplier ReefE;
  // BooleanSupplier ReefF;
  // BooleanSupplier ReefL;
  // BooleanSupplier ReefK;
  // BooleanSupplier ReefJ;
  // BooleanSupplier ReefI;
  // BooleanSupplier ReefH;
  // BooleanSupplier ReefG;
  DoubleSupplier ControllerYAxisSupplier;
  DoubleSupplier ControllerXAxisSupplier;
  DoubleSupplier ControllerZAxisSupplier;
  boolean RightStickSupplier;
  BooleanSupplier ZeroSupplier;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // preferences are initialized IF they don't already exist on the Rio

    Preferences.initBoolean("AntiTipActive", false);

    DataLogManager.start(); // Start logging
    DriverStation.startDataLog(DataLogManager.getLog()); // Joystick Data logging
    /*
     * the following code uses the Xbox Controller Preference to determine our
     * controllers and all our bindings. any time you want to use/create a binding,
     * define a supplier as it in both conditions of this if()else{} code.
     */

    driverControllerXbox = new XboxController(DRIVE_CONTROLLER_ID);
    operatorControllerXbox = new XboxController(OPERATOR_CONTROLLER_ID);

    ControllerXAxisSupplier = () -> driverControllerXbox.getRawAxis(X_AXIS);
    ControllerYAxisSupplier = () -> driverControllerXbox.getRawAxis(Y_AXIS);
    ControllerZAxisSupplier = () -> driverControllerXbox.getRawAxis(Z_AXIS);
    RightStickSupplier = driverControllerXbox.getRightStickButton();

    ZeroGyroSup = driverControllerXbox::getStartButton;

    AlgaeIntakeSup = () -> (driverControllerXbox.getRightTriggerAxis() > 0.1);
    CoralIntakeSup = () -> (driverControllerXbox.getLeftTriggerAxis() > 0.1);
    // AlgaeProcessorOuttakeSup = driverControllerXbox::getLeftBumperButton;
    CoralReefScoreSup = driverControllerXbox::getRightBumperButton;

    ReefHeight1Supplier = driverControllerXbox::getXButton;
    ReefHeight2Supplier = driverControllerXbox::getYButton;
    ReefHeight3Supplier = driverControllerXbox::getBButton;
    ReefHeight4Supplier = driverControllerXbox::getAButton;

    ZeroSupplier = driverControllerXbox::getLeftStickButton;

    removeAlgaeLowSupplier = () -> driverControllerXbox.getPOV() == 180;
    LeftReefLineupSup = () -> operatorControllerXbox.getPOV() == 270;
    RightReefLineupSup = () -> operatorControllerXbox.getPOV() == 90;
    AlgaeProcessorPositionSup = () -> driverControllerXbox.getPOV() == 90;
    removeAlgaeHighSupplier = () -> driverControllerXbox.getPOV() == 0;

    BargeHeightSupplier = () -> (operatorControllerXbox.getLeftTriggerAxis() > 0.1);
    // ReefA =
    // () ->
    // (driverControllerXbox.getRightTriggerAxis() > 0.1)
    // && driverControllerXbox.getLeftBumperButton();
    // ;
    // ReefB =
    // () ->
    // (driverControllerXbox.getRightTriggerAxis() > 0.1)
    // && driverControllerXbox.getRightBumperButton();
    // ;
    // ReefC = () -> driverControllerXbox.getAButton() &&
    // driverControllerXbox.getLeftBumperButton();
    // ReefD = () -> driverControllerXbox.getAButton() &&
    // driverControllerXbox.getRightBumperButton();
    // ReefE = () -> driverControllerXbox.getBButton() &&
    // driverControllerXbox.getLeftBumperButton();
    // ReefF = () -> driverControllerXbox.getBButton() &&
    // driverControllerXbox.getRightBumperButton();
    // ReefL = () -> driverControllerXbox.getXButton() &&
    // driverControllerXbox.getRightBumperButton();
    // ReefK = () -> driverControllerXbox.getXButton() &&
    // driverControllerXbox.getLeftBumperButton();
    // ReefJ = () -> driverControllerXbox.getYButton() &&
    // driverControllerXbox.getRightBumperButton();
    // ReefI = () -> driverControllerXbox.getYButton() &&
    // driverControllerXbox.getLeftBumperButton();
    // ReefH =
    // () ->
    // (driverControllerXbox.getLeftTriggerAxis() > 0.1)
    // && driverControllerXbox.getLeftBumperButton();
    // ReefG =
    // () ->
    // (driverControllerXbox.getLeftTriggerAxis() > 0.1)
    // && driverControllerXbox.getRightBumperButton();

    limelightInit();

    driveTrainInst();

    lightsInst();

    elevatorInst();

    // intakeInst();

    endEffectorInst();

    alignInit();

    configureDriveTrain();

    configureBindings();

    // Configure the trigger bindings
    autoInit();

    NamedCommands.registerCommand(
        "L4",
        new ParallelCommandGroup(
            new ElevatorCommand(elevator, Constants.ElevatorConstants.REEF_LEVEL_4_MILLIMETERS),
            new SequentialCommandGroup(
                new WaitCommand(0.5), // Wait 2 seconds before running the wrist command
                new WristSetpointCommand(endEffector, Constants.EndEffectorConstants.WRIST_L4))));

    NamedCommands.registerCommand(
        "Score Coral",
        new ClawRoller(endEffector, Constants.EndEffectorConstants.EJECT_POWER).withTimeout(1.5));

    NamedCommands.registerCommand(
        "Intake Coral Position",
        new ParallelCommandGroup(
            new ElevatorCommand(
                elevator, Constants.ElevatorConstants.HUMAN_PLAYER_STATION_MILLIMETERS),
            new WristSetpointCommand(
                endEffector, Constants.EndEffectorConstants.WRIST_HUMAN_PLAYER_INTAKE)));

    NamedCommands.registerCommand(
        "Intake Claw Roller",
        new ClawRoller(endEffector, Constants.EndEffectorConstants.INTAKE_POWER));

    NamedCommands.registerCommand("Stop Intake Claw Roller", new ClawRoller(endEffector, 0));
  }

  private void driveTrainInst() {
    driveTrain = new DrivetrainSubsystem();

    defaultDriveCommand =
        new Drive(
            driveTrain,
            ControllerXAxisSupplier, // 1
            ControllerYAxisSupplier, // 0
            ControllerZAxisSupplier, // 4
            RightStickSupplier);
    driveTrain.setDefaultCommand(defaultDriveCommand);
  }

  private void autoInit() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void limelightInit() {
    limelight = Limelight.getInstance();
  }

  private void alignInit() {
    reefAlign = new AutoReefAlignSubsystem(limelight, driveTrain);
  }

  private void lightsInst() {
    lights = new Lights();
    lights.setDefaultCommand(lights.setColor());
    ;
  }

  private void elevatorInst() {
    elevator = new ElevatorSubsystem();
    elevator.setDefaultCommand(new ElevatorManualCommand(elevator));
  }

  // private void intakeInst() {
  // intake = new IntakeSubsystem();
  // }

  private void endEffectorInst() {
    endEffector = new EndEffectorSubsystem();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    SmartDashboard.putData("drivetrain", driveTrain);

    new Trigger(ZeroGyroSup).onTrue(new InstantCommand(driveTrain::zeroHeading));

    // SmartDashboard.putData(new
    // InstantCommand(driveTrain::forceUpdateOdometryWithVision));

    new Trigger(ZeroSupplier)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, 0), new WristSetpointCommand(endEffector, 0)));

    new Trigger(ReefHeight2Supplier)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, Constants.ElevatorConstants.REEF_LEVEL_2_MILLIMETERS),
                new SequentialCommandGroup(
                    new WaitCommand(0.5), // Wait 2 seconds before running the wrist command
                    new WristSetpointCommand(
                        endEffector, Constants.EndEffectorConstants.WRIST_L3))));

    new Trigger(ReefHeight3Supplier)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, Constants.ElevatorConstants.REEF_LEVEL_3_MILLIMETERS),
                new SequentialCommandGroup(
                    new WaitCommand(0.5), // Wait 2 seconds before running the wrist command
                    new WristSetpointCommand(
                        endEffector, Constants.EndEffectorConstants.WRIST_L3))));

    new Trigger(ReefHeight4Supplier)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, Constants.ElevatorConstants.REEF_LEVEL_4_MILLIMETERS),
                new SequentialCommandGroup(
                    new WaitCommand(0.25), // Wait 2 seconds before running the wrist command
                    new WristSetpointCommand(
                        endEffector, Constants.EndEffectorConstants.WRIST_L4))));

    // new Trigger(ReefA).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.A));
    // new Trigger(ReefB).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.B));
    // new Trigger(ReefC).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.C));
    // new Trigger(ReefD).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.D));
    // new Trigger(ReefE).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.E));
    // new Trigger(ReefF).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.F));
    // new Trigger(ReefG).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.G));
    // new Trigger(ReefH).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.H));
    // new Trigger(ReefI).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.I));
    // new Trigger(ReefJ).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.J));
    // new Trigger(ReefK).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.K));
    // new Trigger(ReefL).whileTrue(driveTrain.goToPoint(FieldConstants.Reefs.L));

    // new Trigger(AlgaeIntakeSup)
    // .onTrue(
    // new ParallelCommandGroup(
    // new IntakeSetpointCommand(
    // intake, Constants.IntakeConstants.GROUND_ALGAE_INTAKE_SETPOINT),
    // new IntakePowerCommand(
    // intake, Constants.IntakeConstants.INTAKE_GROUND_ALGAE_POWER)));

    new Trigger(CoralIntakeSup)
        .whileTrue(new ClawRoller(endEffector, Constants.EndEffectorConstants.INTAKE_POWER))
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorCommand(
                    elevator, Constants.ElevatorConstants.HUMAN_PLAYER_STATION_MILLIMETERS),
                new WristSetpointCommand(
                    endEffector, Constants.EndEffectorConstants.WRIST_HUMAN_PLAYER_INTAKE)));

    new Trigger(CoralReefScoreSup)
        .whileTrue(new ClawRoller(endEffector, Constants.EndEffectorConstants.EJECT_POWER));

    // new Trigger(AlgaeProcessorOuttakeSup)
    // .onTrue(
    // new ParallelCommandGroup(
    // new IntakeSetpointCommand(
    // intake, Constants.IntakeConstants.ALGAE_PROCESSOR_SETPOINT),
    // new IntakePowerCommand(
    // intake, Constants.IntakeConstants.EJECT_GROUND_ALGAE_POWER)));

    new Trigger(removeAlgaeHighSupplier)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, Constants.ElevatorConstants.HIGH_ALGAE_HEIGHT),
                new SequentialCommandGroup(
                    new WaitCommand(0.5), // Wait 2 seconds before running the wrist command
                    new WristSetpointCommand(
                        endEffector, Constants.EndEffectorConstants.WRIST_ALGAE_POSITION))))
        .whileTrue(new ClawRoller(endEffector, Constants.EndEffectorConstants.INTAKE_POWER));

    new Trigger(removeAlgaeLowSupplier)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, Constants.ElevatorConstants.LOW_ALGAE_HEIGHT),
                new SequentialCommandGroup(
                    new WaitCommand(0.5), // Wait 2 seconds before running the wrist command
                    new WristSetpointCommand(
                        endEffector, Constants.EndEffectorConstants.WRIST_ALGAE_POSITION))))
        .whileTrue(new ClawRoller(endEffector, Constants.EndEffectorConstants.INTAKE_POWER));

    new Trigger(LeftReefLineupSup).whileTrue(reefAlign.leftReefAlign());

    new Trigger(RightReefLineupSup).whileTrue(reefAlign.rightReefAlign());

    new Trigger(BargeHeightSupplier)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorCommand(elevator, Constants.ElevatorConstants.BARGE_HEIGHT),
                new WristSetpointCommand(
                    endEffector, Constants.EndEffectorConstants.WRIST_BARGE_POSITION)));

    new Trigger(AlgaeProcessorPositionSup)
        .onTrue(
            new ParallelCommandGroup(
                new ElevatorCommand(
                    elevator, Constants.ElevatorConstants.PROCESSOR_HEIGHT_MILLIMETERS),
                new SequentialCommandGroup(
                    new WaitCommand(0.5), // Wait 2 seconds before running the wrist command
                    new WristSetpointCommand(
                        endEffector, Constants.EndEffectorConstants.WRIST_PROCESSOR_POSITION))))
        .whileTrue(new ClawRoller(endEffector, 1.5));
  }

  // Schedule `exampleMethodCommand` when the Xbox controller's B button is
  // pressed,
  // cancelling on release.

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private void configureDriveTrain() {
    try {
      AutoBuilder.configure(
          driveTrain::getPose, // Pose2d supplier
          driveTrain::zeroPose, // Pose2d consumer, used to reset odometry at the beginning of auto
          driveTrain::getChassisSpeeds,
          driveTrain::setChassisSpeeds,
          new PPHolonomicDriveController(
              new com.pathplanner.lib.config.PIDConstants(
                  6, 0, 0), // PID constants to correct for translation error (used to create the X
              // and Y PID controllers)
              new com.pathplanner.lib.config.PIDConstants(
                  5, 0, 0) // PID constants to correct for rotation error (used to create the
              // rotation controller)
              ),
          RobotConfig.fromGUISettings(),
          () -> DriverStation.getAlliance().get().equals(Alliance.Red),
          driveTrain);
    } catch (org.json.simple.parser.ParseException a) {
      System.out.println("got ParseException trying to configure AutoBuilder");
    } catch (IOException b) {
      System.out.println("got IOException thrown trying to configure autobuilder");
    }
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void autonomousInit() {
    SmartDashboard.putNumber("autos ran", SmartDashboard.getNumber("autos ran", 0) + 1);
  }

  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  // private void registerNamedCommands() {
  // Command elevatorResetNamedCommand;

  // elevatorResetNamedCommand =
  // new ElevatorCommand(elevator,
  // Constants.ElevatorConstants.HUMAN_PLAYER_STATION_MILLIMETERS);

  // NamedCommands.registerCommand("ElevatorReset", elevatorResetNamedCommand);
  // }

  public void logPower() {
    for (int i = 0; i < 16; i++) {
      SmartDashboard.putNumber("PDP Current " + i, PDP.getCurrent(i));
    }
  }

  public void teleopInit() {}

  public void teleopPeriodic() {

    SmartDashboard.putData(driveTrain.getCurrentCommand());
  }

  public void robotInit() {}

  public void robotPeriodic() {
    currentAlliance = DriverStation.getAlliance().get();
    SmartDashboard.putString(
        "AlliancePeriodic",
        currentAlliance == null ? "null" : currentAlliance == Alliance.Red ? "Red" : "Blue");
    if (Preferences.getBoolean("Use Limelight", false)) {
      // limelight.updateLoggingWithPoses();
      SmartDashboard.putBoolean(
          "LIMELIGHT/isConnectedFront",
          Limelight.getInstance().isConnected(Vision.APRILTAG_LIMELIGHTA_NAME));
      SmartDashboard.putBoolean(
          "LIMELIGHT/isConnectedBack",
          Limelight.getInstance().isConnected(Vision.APRILTAG_LIMELIGHTB_NAME));
      SmartDashboard.putBoolean(
          "LIMELIGHT/isConnectedC",
          Limelight.getInstance().isConnected(Vision.APRILTAG_LIMELIGHTC_NAME));
    }
  }

  public void disabledPeriodic() {}

  public void disabledInit() {}
}

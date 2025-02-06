// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.DriveConstants.*;
import static frc.robot.settings.Constants.xboxDriver.*;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IndicateLights;
import frc.robot.settings.Constants.Vision;
import frc.robot.settings.ElevatorStates;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
  private final boolean elevatorExists = Preferences.getBoolean("Elevator", true);
  private final boolean lightsExist = Preferences.getBoolean("Lights Exist", true);

  private DrivetrainSubsystem driveTrain;
  private ElevatorSubsystem elevator;

  private Drive defaultDriveCommand;
  private ElevatorCommand elevatorDefaultCommand;

  private XboxController driverControllerXbox;
  private XboxController operatorControllerXbox;

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
  BooleanSupplier AlgaeShooterSup;
  BooleanSupplier ReefHeight1Supplier;
  BooleanSupplier ReefHeight2Supplier;
  BooleanSupplier ReefHeight3Supplier;
  BooleanSupplier ReefHeight4Supplier;
  BooleanSupplier CoralPlaceTeleSupplier;
  BooleanSupplier CoralIntakeHeightSupplier;
  BooleanSupplier ReefA;
  BooleanSupplier ReefB;
  BooleanSupplier ReefC;
  BooleanSupplier ReefD;
  BooleanSupplier ReefE;
  BooleanSupplier ReefF;
  BooleanSupplier ReefL;
  BooleanSupplier ReefK;
  BooleanSupplier ReefJ;
  BooleanSupplier ReefI;
  BooleanSupplier ReefH;
  BooleanSupplier ReefG;
  DoubleSupplier ControllerYAxisSupplier;
  DoubleSupplier ControllerXAxisSupplier;
  DoubleSupplier ControllerZAxisSupplier;
  boolean RightStickSupplier;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // preferences are initialized IF they don't already exist on the Rio

    Preferences.initBoolean("CompBot", true);
    Preferences.initBoolean("Use Limelight", true);
    Preferences.initBoolean("Xbox Controller", true);
    Preferences.initBoolean("Elevator", false);
    Preferences.initBoolean("DrivetrainExists", true);
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

    ControllerXAxisSupplier = () -> -driverControllerXbox.getRawAxis(X_AXIS);
    ControllerYAxisSupplier = () -> -driverControllerXbox.getRawAxis(Y_AXIS);
    ControllerZAxisSupplier = () -> driverControllerXbox.getRawAxis(Z_AXIS);
    RightStickSupplier = driverControllerXbox.getRightStickButton();

    ZeroGyroSup = driverControllerXbox::getStartButton;
    LeftReefLineupSup = driverControllerXbox::getLeftBumperButton;
    RightReefLineupSup = driverControllerXbox::getRightBumperButton;
    SlowFrontSup = () -> driverControllerXbox.getRightTriggerAxis() > 0.1;
    AlgaeIntakeSup = driverControllerXbox::getAButton; // TODO change to actual
    AlgaeShooterSup = driverControllerXbox::getXButton;
    CoralPlaceTeleSupplier = () -> driverControllerXbox.getPOV() == 0;

    ReefHeight1Supplier = () -> driverControllerXbox.getPOV() == 0;
    ReefHeight2Supplier = () -> driverControllerXbox.getPOV() == 90;
    ReefHeight3Supplier = () -> driverControllerXbox.getPOV() == 180;
    ReefHeight4Supplier = () -> driverControllerXbox.getPOV() == 270;
    ReefA =
        () ->
            (driverControllerXbox.getRightTriggerAxis() > 0.1)
                && driverControllerXbox.getLeftBumperButton();
    ;
    ReefB =
        () ->
            (driverControllerXbox.getRightTriggerAxis() > 0.1)
                && driverControllerXbox.getRightBumperButton();
    ;
    ReefC = () -> driverControllerXbox.getAButton() && driverControllerXbox.getLeftBumperButton();
    ReefD = () -> driverControllerXbox.getAButton() && driverControllerXbox.getRightBumperButton();
    ReefE = () -> driverControllerXbox.getBButton() && driverControllerXbox.getLeftBumperButton();
    ReefF = () -> driverControllerXbox.getBButton() && driverControllerXbox.getRightBumperButton();
    ReefL = () -> driverControllerXbox.getXButton() && driverControllerXbox.getRightBumperButton();
    ReefK = () -> driverControllerXbox.getXButton() && driverControllerXbox.getLeftBumperButton();
    ReefJ = () -> driverControllerXbox.getYButton() && driverControllerXbox.getRightBumperButton();
    ReefI = () -> driverControllerXbox.getYButton() && driverControllerXbox.getLeftBumperButton();
    ReefH =
        () ->
            (driverControllerXbox.getLeftTriggerAxis() > 0.1)
                && driverControllerXbox.getLeftBumperButton();
    ReefG =
        () ->
            (driverControllerXbox.getLeftTriggerAxis() > 0.1)
                && driverControllerXbox.getRightBumperButton();
    CoralIntakeHeightSupplier = () -> operatorControllerXbox.getStartButton();

    limelightInit();

    driveTrainInst();

    lightsInst();

    if (elevatorExists) {
      elevatorInst();
    }

    configureDriveTrain();

    configureBindings(); // Configure the trigger bindings
    autoInit();
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
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void limelightInit() {
    limelight = Limelight.getInstance();
  }

  private void lightsInst() {
    lights = new Lights();
    lights.setDefaultCommand(new IndicateLights(lights));
  }

  private void elevatorInst() {
    elevator = new ElevatorSubsystem();
    elevatorDefaultCommand = new ElevatorCommand(elevator, () -> ElevatorStates.HumanPlayer);
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

    // SmartDashboard.putData(new InstantCommand(driveTrain::forceUpdateOdometryWithVision));

    new Trigger(ReefHeight1Supplier)
        .onTrue(
            new InstantCommand(
                () -> RobotState.getInstance().deliveringCoralHeight = ElevatorStates.Reef1));
    new Trigger(ReefHeight2Supplier)
        .onTrue(
            new InstantCommand(
                () -> RobotState.getInstance().deliveringCoralHeight = ElevatorStates.Reef2));
    new Trigger(ReefHeight3Supplier)
        .onTrue(
            new InstantCommand(
                () -> RobotState.getInstance().deliveringCoralHeight = ElevatorStates.Reef3));
    new Trigger(ReefHeight4Supplier)
        .onTrue(
            new InstantCommand(
                () -> RobotState.getInstance().deliveringCoralHeight = ElevatorStates.Reef4));
    new Trigger(CoralIntakeHeightSupplier)
        .onTrue(
            new InstantCommand(
                () -> RobotState.getInstance().deliveringCoralHeight = ElevatorStates.HumanPlayer));
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
                  k_XY_P, k_XY_I,
                  k_XY_D), // PID constants to correct for translation error (used to create the X
              // and Y PID controllers)
              new com.pathplanner.lib.config.PIDConstants(
                  k_THETA_P, k_THETA_I,
                  k_THETA_D) // PID constants to correct for rotation error (used to create the
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

  private void registerNamedCommands() {
    Command elevatorResetNamedCommand;

    if (elevatorExists) {
      elevatorResetNamedCommand =
          new InstantCommand(() -> elevator.setElevatorPosition(ElevatorStates.HumanPlayer));
    } else {
      elevatorResetNamedCommand =
          new InstantCommand(
              () ->
                  System.out.println(
                      "attempted to create named command but subsytem did not exist"));
    }

    NamedCommands.registerCommand("ElevatorReset", elevatorResetNamedCommand);
  }

  public void logPower() {
    for (int i = 0; i < 16; i++) {
      SmartDashboard.putNumber("PDP Current " + i, PDP.getCurrent(i));
    }
  }

  public void teleopInit() {}

  public void teleopPeriodic() {
    if (DrivetrainExists) {
      SmartDashboard.putData(driveTrain.getCurrentCommand());
    }
  }

  public void robotInit() {}

  public void robotPeriodic() {
    currentAlliance = DriverStation.getAlliance().get();
    SmartDashboard.putString(
        "AlliancePeriodic",
        currentAlliance == null ? "null" : currentAlliance == Alliance.Red ? "Red" : "Blue");
    if (Preferences.getBoolean("Use Limelight", false)) {
      limelight.updateLoggingWithPoses();
      SmartDashboard.putBoolean(
          "LIMELIGHT/isConnectedFront",
          Limelight.getInstance().isConnected(Vision.APRILTAG_FRONT_LIMELIGHT));
      SmartDashboard.putBoolean(
          "LIMELIGHT/isConnectedBack",
          Limelight.getInstance().isConnected(Vision.APRILTAG_BACK_LIMELIGHT));
    }
  }

  public void disabledPeriodic() {}

  public void disabledInit() {}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants() {}

  public static final int candleID = 40;

  public static final class DriveConstants {
    public static final int pigeonID = 13;

    public static final COTSTalonFXSwerveConstants
        chosenModule = // TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(
                COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Drivetrain Constants */
    public static final double trackWidth =
        Units.inchesToMeters(28); // TODO: This must be tuned to specific robot
    public static final double wheelBase =
        Units.inchesToMeters(28); // TODO: This must be tuned to specific robot
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 35;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 30;
    public static final int driveCurrentThreshold = 40;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    // public static final double openLoopRamp = 0.175;
    public static final double openLoopRamp = 0.15;
    public static final double closedLoopRamp = 0.05;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity =
        6.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(36.73);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 9;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-152.4902325);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-118);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 12;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-66.1816475);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-145.5);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(167.6);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Auto PID loops
    // twin pid controllers that control the x and y robot movements.
    public static final double k_XY_P = 5; // *2.5;
    public static final double k_XY_I = 0.0;
    public static final double k_XY_D = 0.0;

    public static final double k_THETA_P = 4;
    public static final double k_THETA_I = 5.0;
    public static final double k_THETA_D = 0.0;
    public static final double k_THETA_TOLORANCE_DEGREES = 2.0;
    public static final double k_THETA_TOLORANCE_DEG_PER_SEC = 10;

    public static final double AUTO_AIM_ROBOT_kP = 0.125;
    public static final double AUTO_AIM_ROBOT_kI = 0.00;
    public static final double AUTO_AIM_ROBOT_kD = 0.00;
    public static final double ROBOT_ANGLE_TOLERANCE = 0.5;

    public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
        new PathConstraints(2, 1.5, Math.toRadians(360), Math.toRadians(360));

    public static final Pose2d DRIVE_ODOMETRY_ORIGIN = new Pose2d(5.0, 5.0, new Rotation2d());
  }

  public static final class CTREConfigs {
    // Drive motor.
    public static TalonFXConfiguration getDriveMotorConfig() {
      /** Swerve Drive Motor Configuration */
      /* Motor Inverts and Neutral Mode */
      TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
      swerveDriveFXConfig.MotorOutput.Inverted = Constants.DriveConstants.driveMotorInvert;
      swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.DriveConstants.driveNeutralMode;

      /* Gear Ratio Config */
      swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.DriveConstants.driveGearRatio;

      /* Current Limiting */
      swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
          Constants.DriveConstants.driveEnableCurrentLimit;
      swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit =
          Constants.DriveConstants.driveCurrentLimit;
      // swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold =
      // Constants.Swerve.driveCurrentThreshold;
      // swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold =
      // Constants.Swerve.driveCurrentThresholdTime;

      /* PID Config */
      swerveDriveFXConfig.Slot0.kP = Constants.DriveConstants.driveKP;
      swerveDriveFXConfig.Slot0.kI = Constants.DriveConstants.driveKI;
      swerveDriveFXConfig.Slot0.kD = Constants.DriveConstants.driveKD;

      /* Open and Closed Loop Ramping */
      swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
          Constants.DriveConstants.openLoopRamp;
      swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
          Constants.DriveConstants.openLoopRamp;

      swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
          Constants.DriveConstants.closedLoopRamp;
      swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
          Constants.DriveConstants.closedLoopRamp;

      return swerveDriveFXConfig;
    }

    public static TalonFXConfiguration getSteerMotorConfig() {
      /** Swerve Angle Motor Configurations */
      /* Motor Inverts and Neutral Mode */
      TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
      swerveAngleFXConfig.MotorOutput.Inverted = Constants.DriveConstants.angleMotorInvert;
      swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.DriveConstants.angleNeutralMode;

      /* Gear Ratio and Wrapping Config */
      swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.DriveConstants.angleGearRatio;
      swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

      /* Current Limiting */
      swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
          Constants.DriveConstants.angleEnableCurrentLimit;
      swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit =
          Constants.DriveConstants.angleCurrentLimit;
      // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold =
      // Constants.Swerve.angleCurrentThreshold;
      // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold =
      // Constants.Swerve.angleCurrentThresholdTime;

      /* PID Config */
      swerveAngleFXConfig.Slot0.kP = Constants.DriveConstants.angleKP;
      swerveAngleFXConfig.Slot0.kI = Constants.DriveConstants.angleKI;
      swerveAngleFXConfig.Slot0.kD = Constants.DriveConstants.angleKD;

      return swerveAngleFXConfig;
    }
    //  Steer encoder.
    public static CANcoderConfiguration getSteerEncoderConfig() {
      CANcoderConfiguration steerEncoderConfig = new CANcoderConfiguration();
      steerEncoderConfig.MagnetSensor.SensorDirection = Constants.DriveConstants.cancoderInvert;
      return steerEncoderConfig;
    }

    // Pigeon 2.
    public static Pigeon2Configuration getPigeon2Config() {
      Pigeon2Configuration pigeon2Config = new Pigeon2Configuration();
      pigeon2Config.MountPose.MountPosePitch = 0;
      pigeon2Config.MountPose.MountPoseRoll = 0;
      pigeon2Config.MountPose.MountPoseYaw = 0;
      return pigeon2Config;
    }

    public static final TalonFXConfiguration driveMotorConfig = getDriveMotorConfig();
    public static final TalonFXConfiguration steerMotorConfig = getSteerMotorConfig();
    public static final CANcoderConfiguration steerEncoderConfig = getSteerEncoderConfig();
    public static final Pigeon2Configuration pigeon2Config = getPigeon2Config();
  }

  public final class xboxDriver {
    private xboxDriver() {}

    public static final int DRIVE_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
    /**
     * Left stick X-axis.
     *
     * <p>Left = -1 || Right = 1
     */
    public static final int X_AXIS = 1;
    ;
    /**
     * Left stick Y-axis.
     *
     * <p>Forwards = -1 || Backwards = 1
     */
    public static final int Y_AXIS = 0;
    /**
     * Right stick Z-axis.
     *
     * <p>Left = -1 || Right = 1
     */
    public static final int Z_AXIS = 4;
    /**
     * Right stick Z-rotate.
     *
     * <p>Forwards = -1 || Backwards = 1
     */
    public static final int Z_ROTATE = 4;
    /** Value used to differentiate between angle 0 and rest position. */
    public static final double NO_INPUT = 404;

    public static final double DEADBAND_NORMAL = 0.08;
    public static final double DEADBAND_LARGE = 0.1;
  }

  public final class Field {}

  public final class Vision {
    public static final String APRILTAG_LIMELIGHTA_NAME = "limelight-a";
    public static final String APRILTAG_LIMELIGHTB_NAME = "limelight-b";
    public static final String APRILTAG_LIMELIGHTC_NAME = "limelight-c";

    public static final String LIMELIGHT_SHUFFLEBOARD_TAB = "Vision";

    public static final double ALLOWABLE_POSE_DIFFERENCE = 0.5;
    public static final double MAX_TAG_DISTANCE = 3;

    public static final Translation2d FIELD_CORNER = new Translation2d(16.54, 8.02);

    // how many degrees back is your limelight rotated from perfectly vertical?
    public static final double limelightMountAngleDegrees = 22.0;
    // distance from the center of the Limelight lens to the floor
    public static final double limelightLensHeightInches = 0.233;
    // height of april tags from the floor in meters
    public static final double AprilTagHeight = 1.335;
  }

  public final class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_1_ID = 20;
    public static final int ELEVATOR_MOTOR_2_ID = 21;
    public static final double HUMAN_PLAYER_STATION_MILLIMETERS = 3.9; // 3.9
    public static final double REEF_LEVEL_1_MILLIMETERS = 5; // 23.5
    public static final double REEF_LEVEL_2_MILLIMETERS = 7.3;
    public static final double REEF_LEVEL_3_MILLIMETERS = 11.1;
    public static final double REEF_LEVEL_4_MILLIMETERS = 15.0;
    public static final double HIGH_ALGAE_HEIGHT = 11.0;
    public static final double LOW_ALGAE_HEIGHT = 7.0;
    public static final double PROCESSOR_HEIGHT_MILLIMETERS = 0;
    public static final double ELEVATOR_MILLIMETERS_TO_ROTATIONS = 2531;
    public static final double ELEVATOR_THRESHOLD = 2531;
    public static final double BARGE_HEIGHT = 68;
  }

  public final class EndEffectorConstants {
    public static final int EFFECTOR_MOTOR_1_ID = 30;
    public static final int EFFECTOR_MOTOR_2_ID = 31;

    public static final double WRIST_HUMAN_PLAYER_INTAKE = .65; // 0.65
    public static final double WRIST_L4 = -7.75;
    public static final double WRIST_L3 = -8.4;
    public static final double WRIST_ALGAE_POSITION = -4.7;
    public static final double WRIST_PROCESSOR_POSITION = 4.0;

    public static final double WRIST_BARGE_POSITION = 2531;

    public static final double INTAKE_POWER = 3.5;
    public static final double EJECT_POWER = -3.5;
  }

  public final class IntakeConstants {
    public static final int PIVOT_MOTOR_1_ID = 50;
    public static final int PIVOT_MOTOR_2_ID = 51;
    public static final int PIVOT_MOTOR_3_ID = 52;
    public static final int INTAKE_SPIN_MOTOR_ID = 53;

    public static final double GROUND_ALGAE_INTAKE_SETPOINT = 2531;
    public static final double ALGAE_PROCESSOR_SETPOINT = 2531;
    public static final double INTAKE_GROUND_ALGAE_POWER = 7.5;
    public static final double EJECT_GROUND_ALGAE_POWER = -7.6;
  }

  public final class FieldConstants {
    // used for the autoAngleAtReefCommand
    public static final int REEF_ANGLE_0 = 0;
    public static final int REEF_ANGLE_1 = 60;
    public static final int REEF_ANGLE_2 = 120;
    public static final int REEF_ANGLE_3 = 180;
  }
}

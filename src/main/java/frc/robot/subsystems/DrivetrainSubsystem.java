package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.helpers.AllianceFlipUtil;
import frc.robot.settings.Constants;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSubsystem extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;
  private final SwerveDrivePoseEstimator odometer;
  private final Field2d m_field = new Field2d();
  Limelight limelight;

  // private Vision vision = new Vision();
  // private SwerveDrivePoseEstimator swervePoseEstimator;

  public DrivetrainSubsystem() {
    gyro = new Pigeon2(Constants.DriveConstants.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);
    this.limelight = Limelight.getInstance();
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.DriveConstants.Mod0.constants),
          new SwerveModule(1, Constants.DriveConstants.Mod1.constants),
          new SwerveModule(2, Constants.DriveConstants.Mod2.constants),
          new SwerveModule(3, Constants.DriveConstants.Mod3.constants)
        };

    // use if estimator doesnt work
    // swervePoseEstimator = new
    // SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
    // getGyroYaw(), getModulePositions(), new Pose2d());
    // swervePoseEstimator.addVisionMeasurement(vision.getPose2d(),
    // Timer.getFPGATimestamp());

    odometer =
        new SwerveDrivePoseEstimator(
            Constants.DriveConstants.swerveKinematics,
            getGyroscopeRotation(),
            getModulePositions(),
            new Pose2d());
    odometer.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 99999999));
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> m_field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("Vision/force use limelight", false);
  } // <-- Added missing closing brace for the constructor

  public Command goToPoint(int x, int y) {
    Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(180));
    PathConstraints constraints =
        new PathConstraints(3.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    return AutoBuilder.pathfindToPose(targetPose, constraints);
  }
  /*
   * flips if needed
   */
  public Command goToPoint(Pose2d pose) {
    PathConstraints constraints =
        new PathConstraints(3.0, 2.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    return new ConditionalCommand(
        AutoBuilder.pathfindToPoseFlipped(pose, constraints),
        AutoBuilder.pathfindToPose(pose, constraints),
        AllianceFlipUtil::shouldFlip);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    // Convert the translation and rotation into ChassisSpeeds
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getHeading())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    // Apply anti-tip logic to the chassis speeds
    chassisSpeeds = applyAntiTipLogic(chassisSpeeds);

    // Convert ChassisSpeeds to SwerveModuleStates
    SwerveModuleState[] swerveModuleStates =
        Constants.DriveConstants.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, 0.02));

    // Desaturate wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.maxSpeed);

    // Set the desired state for each module
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Helper method to apply anti-tip logic to the ChassisSpeeds.
   *
   * @param chassisSpeeds The original ChassisSpeeds.
   * @return The adjusted ChassisSpeeds with anti-tip logic applied.
   */
  private ChassisSpeeds applyAntiTipLogic(ChassisSpeeds chassisSpeeds) {

    double pitch = getPigeonPitch();
    double roll = getPigeonRoll();

    // Adjust the chassis speeds based on pitch and roll
    if (roll > 3) {
      chassisSpeeds.vxMetersPerSecond += (roll / 10);
    } else if (roll < -3) {
      chassisSpeeds.vxMetersPerSecond += (-roll / 10);
    }

    if (pitch > 3) {
      chassisSpeeds.vyMetersPerSecond += (pitch / 10);
    } else if (pitch < -3) {
      chassisSpeeds.vyMetersPerSecond += (-pitch / 10);
    }

    return chassisSpeeds;
  }
  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void setGyroscope(double angleDeg) {
    zeroPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(angleDeg)));
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
    // return swervePoseEstimator.getEstimatedPosition();
  }

  // public Pose2d getEstimatedPose() {
  // return swervePoseEstimator.getEstimatedPosition();
  // }

  public void setPose(Pose2d pose) {
    odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public void zeroPose(Pose2d pose) {
    odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /*
   * Logs important data for the drivetrain
   */
  private void logDrivetrainData() {
    SmartDashboard.putNumber("DRIVETRAIN/Robot Angle", getOdometryRotation().getDegrees());
    SmartDashboard.putString("DRIVETRAIN/Robot Location", getPose().getTranslation().toString());
    // SmartDashboard.putString("DRIVETRAIN/Position", getPose());
    SmartDashboard.putNumber("DRIVETRAIN/forward speed", getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber(
        "DRIVETRAIN/rotational speed", Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));
    SmartDashboard.putNumber(
        "DRIVETRAIN/gyroscope rotation degrees", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber(
        "DRIVETRAIN/degrees per second", Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));

    Logger.recordOutput("MyStates", getModuleStates());
    Logger.recordOutput("Position", odometer.getEstimatedPosition());
    Logger.recordOutput("Gyro", getGyroscopeRotation());
  }

  public void setHeading(Rotation2d heading) {
    odometer.resetPosition(
        getGyroscopeRotation(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), heading));
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new
    // Pose2d(getPose().getTranslation(), heading));
  }

  /**
   * @return a rotation2D of the angle according to the odometer
   */
  public Rotation2d getOdometryRotation() {
    return odometer.getEstimatedPosition().getRotation();
  }

  /**
   * Provide the odometry a vision pose estimate, only if there is a trustworthy pose available.
   *
   * <p>Each time a vision pose is supplied, the odometry pose estimation will change a little,
   * larger pose shifts will take multiple calls to complete.
   */
  public void updateOdometryWithVision() {
    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.APRILTAG_LIMELIGHTA_NAME,
        odometer.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.APRILTAG_LIMELIGHTB_NAME,
        odometer.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        Constants.Vision.APRILTAG_LIMELIGHTC_NAME,
        odometer.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);

    PoseEstimate estimate = limelight.getTrustedPose();
    if (estimate != null) {
      boolean doRejectUpdate = false;
      if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) {
        doRejectUpdate = true;
      }
      if (estimate.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        odometer.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
      }
      RobotState.getInstance().LimelightsUpdated = true;
    } else {
      RobotState.getInstance().LimelightsUpdated = false;
    }
  }

  /**
   * Set the odometry using the current apriltag estimate, disregarding the pose trustworthyness.
   *
   * <p>You only need to run this once for it to take effect.
   */
  public void forceUpdateOdometryWithVision() {
    PoseEstimate estimate = limelight.getTrustedPose();
    if (estimate != null) {
      zeroPose(estimate.pose);
    } else {
      System.err.println(
          "No valid limelight estimate to reset from. (Drivetrain.forceUpdateOdometryWithVision)");
    }
  }

  public void zeroHeading() {
    odometer.resetPosition(
        getGyroscopeRotation(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new
    // Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public double getPigeonPitch() {
    double pitch = gyro.getPitch().getValueAsDouble();
    return pitch;
  }

  public double getPigeonRoll() {
    double roll = gyro.getRoll().getValueAsDouble();
    return roll;
  }

  public Rotation2d getGyroscopeRotation() {
    return gyro.getRotation2d();
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public double getGyroDouble() {
    return gyro.getYaw().getValueAsDouble();
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] currentModuleState = new SwerveModuleState[this.mSwerveMods.length];
    for (int i = 0; i < this.mSwerveMods.length; i++) {
      currentModuleState[i] = mSwerveMods[i].getState();
    }
    return Constants.DriveConstants.swerveKinematics.toChassisSpeeds(currentModuleState);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(Constants.DriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void stopModules() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stop();
    }
  }

  // This is the odometry section
  /** Updates the odometry */
  public void updateOdometry() {
    odometer.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), getModulePositions());
  }

  @Override
  public void periodic() {
    updateOdometry(); // this is used if estimator doesnt work
    // swervePoseEstimator.update(getGyroYaw(), getModulePositions());
    SmartDashboard.putNumber("pose2d X", getPose().getX());
    SmartDashboard.putNumber("pose2d Y", getPose().getY());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    SmartDashboard.putNumber("Gyro rotation", getGyroDouble());
    SmartDashboard.putNumber("rotation pose", getGyroscopeRotation().getDegrees());

    // sets the robot orientation for each of the limelights, which is required for
    // the
    // if (Preferences.getBoolean("Use Limelight", false)) {
    //   if (SmartDashboard.getBoolean("Vision/force use limelight", false)) {
    //     forceUpdateOdometryWithVision();
    //   } else {
    //     updateOdometryWithVision();
    //   }
    // } else {
    //   RobotState.getInstance().LimelightsUpdated = false;
    // }

    m_field.setRobotPose(odometer.getEstimatedPosition());
    logDrivetrainData();
    RobotState.getInstance().odometerOrientation = getOdometryRotation().getDegrees();
  }
}

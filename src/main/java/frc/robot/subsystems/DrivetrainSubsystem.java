package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;

  // private Vision vision = new Vision();
  // private SwerveDrivePoseEstimator swervePoseEstimator;

  public DrivetrainSubsystem() {
    gyro = new Pigeon2(Constants.DriveConstants.pigeonID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.DriveConstants.Mod0.constants),
          new SwerveModule(1, Constants.DriveConstants.Mod1.constants),
          new SwerveModule(2, Constants.DriveConstants.Mod2.constants),
          new SwerveModule(3, Constants.DriveConstants.Mod3.constants)
        };

    swerveOdometry =
        new SwerveDriveOdometry(
            Constants.DriveConstants.swerveKinematics,
            getGyroYaw(),
            getModulePositions()); // use if estimator doesnt work
    // swervePoseEstimator =  new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
    // getGyroYaw(), getModulePositions(), new Pose2d());
    // swervePoseEstimator.addVisionMeasurement(vision.getPose2d(), Timer.getFPGATimestamp());
  } // <-- Added missing closing brace for the constructor

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, Boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.DriveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getHeading()
                    // getGyroYaw()
                    )
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
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
    return swerveOdometry.getPoseMeters();
    // return swervePoseEstimator.getEstimatedPosition();
  }

  // public Pose2d getEstimatedPose() {
  //     return swervePoseEstimator.getEstimatedPosition();
  // }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public void zeroPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    swerveOdometry.resetPosition(
        getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new
    // Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    swerveOdometry.resetPosition(
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
    // swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new
    // Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Rotation2d getGyroYaw() {
    return gyro.getRotation2d();
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

  @Override
  public void periodic() {
    swerveOdometry.update(
        getGyroYaw(), getModulePositions()); // this is used if estimator doesnt work
    // swervePoseEstimator.update(getGyroYaw(), getModulePositions());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}

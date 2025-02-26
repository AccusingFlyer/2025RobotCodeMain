package frc.robot.subsystems;

import static frc.robot.settings.Constants.Vision.APRILTAG_LIMELIGHTC_NAME;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoReefAlignSubsystem extends SubsystemBase {

  public Limelight Limelight;
  public DrivetrainSubsystem Drive;
  // public EndEffectorSubsystem endEffector;

  public double leftReefOffset = 10;
  public double rightReefOffset = -10;

  public double desiredYOffset = 5.0;

  public AutoReefAlignSubsystem(
      Limelight Limelight, DrivetrainSubsystem Drive /*EndEffectorSubsystem endEffector */) {
    this.Limelight = Limelight;
    this.Drive = Drive;
    // this.endEffector = endEffector;
  }

  public Command leftReefAlign() {
    return run(
        () -> {
          NetworkTable table = NetworkTableInstance.getDefault().getTable(APRILTAG_LIMELIGHTC_NAME);
          NetworkTableEntry pipeline = table.getEntry("pipeline");
          table.getEntry("ledMode").setNumber(3);
          pipeline.setNumber(0);

          // check against 'tv' before aligning
          double tv = table.getEntry("tv").getDouble(0);
          double tx = table.getEntry("tx").getDouble(0);
          double ty = table.getEntry("ty").getDouble(0);

          double[] poseArray =
              table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

          double angleError = 0.0;
          if (poseArray.length >= 6) {
            angleError = poseArray[5];
          }

          if (tv < 0.999) {
            Drive.drive(new Translation2d(0, 0), 0, false, true);
          }

          double KpStrafe = 0.02;
          double kpForward = 0.02;
          double kpRotation = 0.02;

          double horizontalError = tx - leftReefOffset;
          double forwardError = ty - desiredYOffset;

          double strafeSpeed = KpStrafe * horizontalError;
          double forwardSpeed = kpForward * forwardError;
          double rotSpeed = kpRotation * angleError;

          strafeSpeed = MathUtil.clamp(strafeSpeed, -0.2, 0.2);
          forwardSpeed = MathUtil.clamp(strafeSpeed, -0.2, 0.2);
          rotSpeed = MathUtil.clamp(strafeSpeed, -0.3, 0.3);

          // double yawCalculated = (Math.signum(Drive.m_gyro.getYaw()) * Math.PI) -
          // (Math.toRadians(Drive.m_gyro.getYaw()));

          // double kpTurn = 0.25;
          Drive.drive(new Translation2d(forwardSpeed, strafeSpeed), rotSpeed, false, true);

          // if (Math.abs(Math.toDegrees(yawCalculated)) > 1 || Math.abs(xError) > .25){
          // rot = MathUtil.clamp((kpTurn * yawCalculated) + .05, -0.25, 0.25);

          // rotSpeed = (KpStrafe * (angleError));
          // ySpeed =
          //     -(KpStrafe
          //         * (leftReefOffset
          //             - tx)); // tx = horizontal error, strafe direction in robot coordinates
          // xSpeed = (KpStrafe * (ty));
          // }

          // rotSpeed = (KpStrafe * (angleError));
          // ySpeed =
          //     -(KpStrafe * (tx)); // tx = horizontal error, strafe direction in robot coordinates
          // xSpeed = (KpStrafe * (ty));

          // if (rotSpeed > 0.25) {
          //   rotSpeed = 0.5;
          // } else if (rotSpeed < -0.25) {
          //   rotSpeed = -0.25;
          // }

          // if (xSpeed > 0.75) {
          //   xSpeed = 0.75;
          // } else if (xSpeed < -0.75) {
          //   xSpeed = -0.75;
          // }
          // if (ySpeed > 0.1) {
          //   ySpeed = 0.1;
          // } else if (ySpeed < -0.1) {
          //   ySpeed = -0.1;
          // }

          // if (tv < 0.9999) {
          //   xSpeed = 0;
          //   ySpeed = 0;
          //   rotSpeed = 0;
          // }

          // Drive.drive(new Translation2d(0, ySpeed), rotSpeed, false, true);

          // if ((tv > 0.9999)
          //     && (Math.abs(rotSpeed) < 0.025)
          //     && (Math.abs(xSpeed) < 0.025)
          //     && (Math.abs(ySpeed) < 0.025)) {
          //   // Commands.runOnce(() -> {

          //   // }, Intake);
          // }

          // debug test
          System.out.printf("Rot, X, Y Speeds");
          System.out.printf("%f\n", rotSpeed);
          System.out.printf("%f\n", forwardSpeed);
          System.out.printf("%f\n", strafeSpeed);
        });
  }

  public Command rightReefAlign() {
    return run(
        () -> {
          NetworkTable table = NetworkTableInstance.getDefault().getTable(APRILTAG_LIMELIGHTC_NAME);
          NetworkTableEntry pipeline = table.getEntry("pipeline");
          table.getEntry("ledMode").setNumber(3);
          pipeline.setNumber(0);

          // check against 'tv' before aligning
          double tv = table.getEntry("tv").getDouble(0);
          double tx = table.getEntry("tx").getDouble(0);
          double ty = table.getEntry("ty").getDouble(0);

          double[] poseArray =
              table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

          double angleError = 0.0;
          if (poseArray.length >= 6) {
            angleError = poseArray[5];
          }

          if (tv < 0.999) {
            Drive.drive(new Translation2d(0, 0), 0, false, true);
          }

          double KpStrafe = 0.02;
          double kpForward = 0.02;
          double kpRotation = 0.02;

          double horizontalError = tx - rightReefOffset;
          double forwardError = ty - desiredYOffset;

          double strafeSpeed = KpStrafe * horizontalError;
          double forwardSpeed = kpForward * forwardError;
          double rotSpeed = kpRotation * angleError;

          strafeSpeed = MathUtil.clamp(strafeSpeed, -0.2, 0.2);
          forwardSpeed = MathUtil.clamp(strafeSpeed, -0.2, 0.2);
          rotSpeed = MathUtil.clamp(strafeSpeed, -0.3, 0.3);

          // double yawCalculated = (Math.signum(Drive.m_gyro.getYaw()) * Math.PI) -
          // (Math.toRadians(Drive.m_gyro.getYaw()));

          // double kpTurn = 0.25;
          Drive.drive(new Translation2d(forwardSpeed, strafeSpeed), rotSpeed, false, true);

          // if (Math.abs(Math.toDegrees(yawCalculated)) > 1 || Math.abs(xError) > .25){
          // rot = MathUtil.clamp((kpTurn * yawCalculated) + .05, -0.25, 0.25);

          // rotSpeed = (KpStrafe * (angleError));
          // ySpeed =
          //     -(KpStrafe
          //         * (leftReefOffset
          //             - tx)); // tx = horizontal error, strafe direction in robot coordinates
          // xSpeed = (KpStrafe * (ty));
          // }

          // rotSpeed = (KpStrafe * (angleError));
          // ySpeed =
          //     -(KpStrafe * (tx)); // tx = horizontal error, strafe direction in robot coordinates
          // xSpeed = (KpStrafe * (ty));

          // if (rotSpeed > 0.25) {
          //   rotSpeed = 0.5;
          // } else if (rotSpeed < -0.25) {
          //   rotSpeed = -0.25;
          // }

          // if (xSpeed > 0.75) {
          //   xSpeed = 0.75;
          // } else if (xSpeed < -0.75) {
          //   xSpeed = -0.75;
          // }
          // if (ySpeed > 0.1) {
          //   ySpeed = 0.1;
          // } else if (ySpeed < -0.1) {
          //   ySpeed = -0.1;
          // }

          // if (tv < 0.9999) {
          //   xSpeed = 0;
          //   ySpeed = 0;
          //   rotSpeed = 0;
          // }

          // Drive.drive(new Translation2d(0, ySpeed), rotSpeed, false, true);

          // if ((tv > 0.9999)
          //     && (Math.abs(rotSpeed) < 0.025)
          //     && (Math.abs(xSpeed) < 0.025)
          //     && (Math.abs(ySpeed) < 0.025)) {
          //   // Commands.runOnce(() -> {

          //   // }, Intake);
          // }

          // debug test
          System.out.printf("Rot, X, Y Speeds");
          System.out.printf("%f\n", rotSpeed);
          System.out.printf("%f\n", forwardSpeed);
          System.out.printf("%f\n", strafeSpeed);
        });
  }
}

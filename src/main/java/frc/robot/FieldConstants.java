package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.875);
  public static final double fieldWidth = Units.inchesToMeters(317);

  public static final class CoralStations {

    public static final Pose2d FarRightCoralPickup =
        new Pose2d(1.606, 0.71, Rotation2d.fromDegrees(143.67));
    public static final Pose2d NearRightCoralPickup =
        new Pose2d(0.755, 1.286, Rotation2d.fromDegrees(143.67));
  }

  public static final class Reefs {
    // Corrected Pose2d values (same X and Y, updated rotation)
    public static final Pose2d A = new Pose2d(3.201, 4.223, Rotation2d.fromDegrees(-90));

    public static final Pose2d B = new Pose2d(3.177, 3.743, Rotation2d.fromDegrees(-90));

    public static final Pose2d C = new Pose2d(3.68, 2.976, Rotation2d.fromDegrees(145));
    public static final Pose2d D = new Pose2d(3.992, 2.796, Rotation2d.fromDegrees(150));

    public static final Pose2d E = new Pose2d(4.975, 2.796, Rotation2d.fromDegrees(-150));

    public static final Pose2d F = new Pose2d(5.335, 2.976, Rotation2d.fromDegrees(-145));

    public static final Pose2d G = new Pose2d(5.79, 3.779, Rotation2d.fromDegrees(-90));

    public static final Pose2d H = new Pose2d(1.0, 4.223, Rotation2d.fromDegrees(-90));

    public static final Pose2d I = new Pose2d(5.311, 5.062, Rotation2d.fromDegrees(150));

    public static final Pose2d J = new Pose2d(4.975, 5.254, Rotation2d.fromDegrees(150));

    public static final Pose2d K = new Pose2d(4.04, 5.254, Rotation2d.fromDegrees(-150));

    public static final Pose2d L = new Pose2d(3.68, 5.098, Rotation2d.fromDegrees(-150));
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
}

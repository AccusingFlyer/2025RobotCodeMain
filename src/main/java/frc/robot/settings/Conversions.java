package frc.robot.settings;

public class Conversions {

  /**
   * @param wheelRPS Wheel Velocity: (in Rotations per Second)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Velocity: (in Meters per Second)
   */
  public static double RPSToMPS(double wheelRPS, double circumference) {
    double wheelMPS = wheelRPS * circumference;
    return wheelMPS;
  }

  /**
   * @param wheelMPS Wheel Velocity: (in Meters per Second)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Velocity: (in Rotations per Second)
   */
  public static double MPSToRPS(double wheelMPS, double circumference) {
    double wheelRPS = wheelMPS / circumference;
    return wheelRPS;
  }

  /**
   * @param wheelRotations Wheel Position: (in Rotations)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Distance: (in Meters)
   */
  public static double rotationsToMeters(double wheelRotations, double circumference) {
    double wheelMeters = wheelRotations * circumference;
    return wheelMeters;
  }

  /**
   * @param wheelMeters Wheel Distance: (in Meters)
   * @param circumference Wheel Circumference: (in Meters)
   * @return Wheel Position: (in Rotations)
   */
  public static double metersToRotations(double wheelMeters, double circumference) {
    double wheelRotations = wheelMeters / circumference;
    return wheelRotations;
  }

  public static double falconToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * 2048.0));
  }

  public static double degreesToFalcon(double degrees, double gearRatio) {
    double ticks = degrees / (360.0 / (gearRatio * 2048.0));
    return ticks;
  }
}

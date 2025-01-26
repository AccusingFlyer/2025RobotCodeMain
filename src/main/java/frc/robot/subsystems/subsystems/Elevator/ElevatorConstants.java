package frc.robot.subsystems.subsystems.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants {
  public static final int LEFT_MOTOR_CANID = 12;
  public static final int RIGHT_MOTOR_CANID = 13;

  public static final Distance POSITION_TOLERANCE = Meters.of(.1);
  public static final LinearVelocity VELOCITY_TOLERANCE = MetersPerSecond.of(.1);
  public static final Constraints TRAPEZOID_PROFILE_CONSTRAINTS =
      new TrapezoidProfile.Constraints(1, .5);
  public static final LinearVelocity ZEROING_VELOCITY = MetersPerSecond.of(0.25);
  public static final Current ZEROING_CURRENT_LIMIT = Amps.of(10.0);

  // TODO: Note for MahiDaMan, All of these are RANDOM values, once the robot is built GET REAL ONES
  public static final Distance L4_HEIGHT = Meters.of(2.5146);
  public static final Distance L3_HEIGHT = Meters.of(1.54305);
  public static final Distance L2_HEIGHT = Meters.of(1.54305);
  public static final Distance L1_HEIGHT = Meters.of(1);
  public static final Distance IDLE_HEIGHT = Meters.of(0.1);
  public static final Distance ALGAE_LOW_HEIGHT = Meters.of(1);
  public static final Distance ALGAE_HIGH_HEIGHT = Meters.of(1);
  public static final Distance ALGAE_PROCESSOR_HEIGHT = Meters.of(1);

  public static final Distance METERS_PER_ROTATION =
      Meters.of(Sim.GEARING * (2 * Math.PI * Sim.DRUM_RADIUS.in(Meters)));

  public static class Sim {

    // TODO change all of these values when new robot is done
    public static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);
    public static final double GEARING = 5.5;
    public static final Mass CARRIAGE_MASS = Pounds.of(25);
    public static final Distance DRUM_RADIUS =
        Inches.of(2.3); // Random value cause chaska dont have a freaking bot
    public static final Distance MIN_HEIGHT = Inches.of(40.5);
    public static final Distance MAX_HEIGHT = Inches.of(93.5);
    public static final boolean SIMULATE_GRAVITY = true;
    public static final Distance STARTING_HEIGHT = Inches.of(37);

    // TODO Tune once we get new values, this has Izone so I didn't make it a supplier
    public static final PIDConstants PROFILED_PID_CONSTANTS = new PIDConstants(2, 0, 0, 0);
  }

  public static class Real {

    public static final boolean LEFT_INVERTED = false;
    public static final NeutralModeValue LEFT_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final boolean LEFT_STRATOR_CURRENT_LIMIT_ENABLED = true;
    public static final Current LEFT_STRATOR_CURRENT_LIMIT = Amps.of(30);

    public static final boolean RIGHT_INVERTED = false;
    public static final NeutralModeValue RIGHT_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final boolean RIGHT_STRATOR_CURRENT_LIMIT_ENABLED = true;
    public static final Current RIGHT_STRATOR_CURRENT_LIMIT = Amps.of(30);

    public static final PIDConstants PROFILED_PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
  }
}

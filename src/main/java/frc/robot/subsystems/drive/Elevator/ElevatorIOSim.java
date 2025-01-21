package frc.robot.subsystems.drive.Elevator;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.drive.Elevator.ElevatorConstants.*;
import static frc.robot.subsystems.drive.Elevator.ElevatorConstants.Sim.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.drive.Elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOSim implements ElevatorIO {
  private ProfiledPIDController pidController;
  private ElevatorFeedforward ffcontroller;
  private ElevatorSim elevatorSim;

  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private TalonFXSimState leftMotorSim;
  private TalonFXSimState rightMotorSim;

  private double appliedVoltage;
  private boolean zeroed;
  public double metersPerRotation;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            GEARBOX,
            GEARING,
            CARRIAGE_MASS.in(Kilogram),
            DRUM_RADIUS.in(Meters),
            MIN_HEIGHT.in(Meters),
            MAX_HEIGHT.in(Meters),
            SIMULATE_GRAVITY,
            STARTING_HEIGHT.in(Meters));

    pidController =
        new ProfiledPIDController(
            PROFILED_PID_CONSTANTS.kP,
            PROFILED_PID_CONSTANTS.kI,
            PROFILED_PID_CONSTANTS.kD,
            ElevatorConstants.TRAPEZOID_PROFILE_CONSTRAINTS);
    pidController.setTolerance(
        ElevatorConstants.POSITION_TOLERANCE.in(Meters),
        ElevatorConstants.VELOCITY_TOLERANCE.in(MetersPerSecond));
    pidController.setIZone(PROFILED_PID_CONSTANTS.iZone);

    ffcontroller = new ElevatorFeedforward(0, 0, 0, 0);
    zeroed = false;

    leftMotor = new TalonFX(LEFT_MOTOR_CANID);
    rightMotor = new TalonFX(RIGHT_MOTOR_CANID);
    leftMotorSim = leftMotor.getSimState();
    rightMotorSim = rightMotor.getSimState();

    metersPerRotation = METERS_PER_ROTATION.in(Meters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);

    inputs.currentElevatorHeight = elevatorSim.getPositionMeters() * METERS_PER_ROTATION.in(Meters);
    inputs.elevatorHeightSetpoint = pidController.getSetpoint().position;
    inputs.elevatorHeightGoalpoint = pidController.getGoal().position;

    inputs.elevatorVelocity =
        elevatorSim.getVelocityMetersPerSecond() * METERS_PER_ROTATION.in(Meters);
    inputs.elevatorVelocitySetpoint = pidController.getSetpoint().velocity;
    inputs.elevatorVelocityGoalpoint = pidController.getGoal().velocity;

    inputs.leftMotorVoltInput = appliedVoltage;
    inputs.rightMotorVoltInput = appliedVoltage;
    inputs.elevatorZeroed = zeroed;

    leftMotorSim.setRawRotorPosition(
        elevatorSim.getPositionMeters() / METERS_PER_ROTATION.in(Meters));
    leftMotorSim.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond() / METERS_PER_ROTATION.in(Meters));
    rightMotorSim.setRawRotorPosition(
        -elevatorSim.getPositionMeters()
            / METERS_PER_ROTATION.in(Meters)); // negative bc right is inversed (probably)
    rightMotorSim.setRotorVelocity(
        -elevatorSim.getVelocityMetersPerSecond() / METERS_PER_ROTATION.in(Meters));
  }

  // @Override
  // public void setVoltage(double volts) {
  //   appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  // }

  @Override
  public void setHeightGoalpoint(double height) {
    pidController.setGoal(height);
  }

  public void runElevator() {
    appliedVoltage =
        pidController.calculate(elevatorSim.getPositionMeters() * METERS_PER_ROTATION.in(Meters))
            + ffcontroller.calculate(pidController.getSetpoint().velocity);
    elevatorSim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void zero() {
    zeroed = true;
  }

  // @Override
  // public boolean isZeroed() {
  // 	return zeroed;
  // }
}

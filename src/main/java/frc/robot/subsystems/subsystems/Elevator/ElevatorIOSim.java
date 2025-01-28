package frc.robot.subsystems.subsystems.Elevator;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.subsystems.Elevator.ElevatorConstants.*;
import static frc.robot.subsystems.subsystems.Elevator.ElevatorConstants.Sim.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

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

    leftMotor
        .getConfigurator()
        .apply(new FeedbackConfigs().withSensorToMechanismRatio(METERS_PER_ROTATION.in(Meters)));
    // FIXME: This should probably be done by having right motor "follow" the left
    // motor instead of
    // individually configing both
    rightMotor
        .getConfigurator()
        .apply(new FeedbackConfigs().withSensorToMechanismRatio(METERS_PER_ROTATION.in(Meters)));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.currentElevatorHeight = elevatorSim.getPositionMeters();
    inputs.elevatorHeightSetpoint = pidController.getSetpoint().position;
    inputs.elevatorHeightGoalpoint = pidController.getGoal().position;

    inputs.elevatorVelocity = elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorVelocitySetpoint = pidController.getSetpoint().velocity;
    inputs.elevatorVelocityGoalpoint = pidController.getGoal().velocity;

    inputs.leftMotorVoltInput = appliedVoltage;
    inputs.rightMotorVoltInput = appliedVoltage;
    inputs.elevatorZeroed = zeroed;

    leftMotorSim.setRawRotorPosition(elevatorSim.getPositionMeters());
    leftMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond());
    rightMotorSim.setRawRotorPosition(
        -elevatorSim.getPositionMeters()); // negative bc right is inversed (probably)
    rightMotorSim.setRotorVelocity(-elevatorSim.getVelocityMetersPerSecond());
  }

  @Override
  public void setHeightGoalpoint(double height) {
    pidController.setGoal(height);
  }

  public void runElevator() {
    double batteryVoltage = RobotController.getBatteryVoltage();
    double pidOutput = pidController.calculate(elevatorSim.getPositionMeters());
    // double ffOutput = // TODO: Feedforward shouldn't be for velocity. It is
    // basically a constant
    // output voltage just enough to hold the elevator up. Also your constants are
    // all currently 0.

    appliedVoltage = batteryVoltage * pidOutput;

    leftMotorSim.setSupplyVoltage(batteryVoltage);
    rightMotorSim.setSupplyVoltage(batteryVoltage);

    // While this may seem redundant (and it is if you're only doing voltage), it
    // allows you to
    // change it to a different type of control (like motion magic)
    // I highly recommend that you swap the profiled PID controller for internal
    // motion magic.
    leftMotor.setControl(new VoltageOut(appliedVoltage));
    rightMotor.setControl(new VoltageOut(appliedVoltage));

    elevatorSim.setInputVoltage(batteryVoltage);
    elevatorSim.setInput(leftMotorSim.getMotorVoltage());
    elevatorSim.update(0.02);
  }

  @Override
  public void zero() {
    zeroed = true;
  }

  // @Override
  // public boolean isZeroed() {
  // return zeroed;
  // }
}

package frc.robot.subsystems.drive.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public String stateString;
    public double currentElevatorHeight;
    public double elevatorHeightSetpoint;
    public double elevatorHeightGoalpoint;
    public double elevatorVelocity;
    public double elevatorVelocitySetpoint;
    public double elevatorVelocityGoalpoint;
    public double leftMotorVoltInput;
    public double rightMotorVoltInput;
    public boolean elevatorZeroed;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setHeightGoalpoint(double height) {}

  public default void runElevator() {}

  public default void zero() {}
}

package frc.robot.subsystems.drive.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
  }

  public Command runtoL4() {
    return runOnce(
        () -> {
          io.setHeightGoalpoint(2.5146);
        });
  }

  public Command runtoL3() {
    return runOnce(
        () -> {
          io.setHeightGoalpoint(1.54305);
          io.runElevator();
        });
  }

  public Command runtoL2() {
    return runOnce(
        () -> {
          io.setHeightGoalpoint(1.25);
          io.runElevator();
        });
  }

  public Command runtoL1() {
    return runOnce(
        () -> {
          io.setHeightGoalpoint(1);
          io.runElevator();
        });
  }

  public Command idleElevatorPos() {
    return runOnce(
        () -> {
          io.setHeightGoalpoint(0.1);
          io.runElevator();
        });
  }
}

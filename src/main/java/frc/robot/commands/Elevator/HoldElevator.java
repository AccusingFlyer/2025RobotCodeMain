package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class HoldElevator extends Command {
  private final ElevatorSubsystem elevator;

  private PIDController pidControllerAv = new PIDController(0.025, 0.0, 0.00);
  private PIDController pidController1 = new PIDController(0.025, 0.0, 0);
  private PIDController pidController2 = new PIDController(0.025, 0.0, 0);

  private double holdPosition;

  public HoldElevator(ElevatorSubsystem elevator) {
    this.elevator = elevator;
    addRequirements(elevator);

    pidControllerAv.setIZone(30);
    pidController1.setIZone(30);
    pidController2.setIZone(30);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    holdPosition = -elevator.getEncoderRight();

    pidControllerAv.setSetpoint(holdPosition);
    pidController1.setSetpoint(holdPosition);
    pidController2.setSetpoint(holdPosition);
  }
}

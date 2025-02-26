// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  ElevatorSubsystem elevator;

  private double setpoint;

  private PIDController pidControllerAv = new PIDController(0.025, 0.0, 0.00);
  private PIDController pidController1 = new PIDController(0.025, 0.0, 0);
  private PIDController pidController2 = new PIDController(0.025, 0.0, 0);

  ElevatorFeedforward feedforward = new ElevatorFeedforward(0.3, 0.3, 1, 1);

  public ElevatorCommand(ElevatorSubsystem elevator, double setpoint) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    // addRequirements(elevator);
    pidControllerAv.setIZone(30);
    pidController1.setIZone(30);
    pidController2.setIZone(30);

    pidControllerAv.setSetpoint(setpoint);
    pidController1.setSetpoint(setpoint);
    pidController2.setSetpoint(setpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController1.reset();
    pidController2.reset();

    elevator.setNeutralMode(NeutralModeValue.Coast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed1 = pidController1.calculate(elevator.getEncoderLeft());
    double speed2 = pidController2.calculate(elevator.getEncoderRight());

    double averagePosition = ((Math.abs(speed1) + speed2) / 2.0);

    double speed = pidControllerAv.calculate(averagePosition);

    // System.out.println("PID Output (Speed): " + speed);

    elevator.setMotors(speed, speed);

    // elevator.setElevatorPosition(averagePosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double feedForwardValue = feedforward.calculate(0);
    elevator.setMotors(feedForwardValue, feedForwardValue);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController1.atSetpoint();
  }
}

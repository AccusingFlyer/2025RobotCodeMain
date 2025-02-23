// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  ElevatorSubsystem elevator;

  private double height;

  private PIDController pidController1 = new PIDController(0.025, 0.0, 0);
  private PIDController pidController2 = new PIDController(0.025, 0.0, 0);
  /**
   * Creates a new ElevatorCommand.
   *
   * @param elevator elevator subsystem
   * @param level level of reef, human player station is 0.
   */
  public ElevatorCommand(ElevatorSubsystem elevator, double height) {
    this.elevator = elevator;
    this.height = height;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.

    pidController1.setIZone(30);
    pidController2.setIZone(30);
    pidController1.setSetpoint(height);
    pidController2.setSetpoint(height);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController1.reset();
    pidController2.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed1 = pidController1.calculate(-elevator.getEncoderLeft(), height);
    double speed2 = pidController2.calculate(elevator.getEncoderRight(), height);

    elevator.setMotors(speed1, speed2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController1.atSetpoint() && pidController2.atSetpoint();
  }
}

package frc.robot.commands.EndEffector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class WristSetpointCommand extends Command {
  private EndEffectorSubsystem wrist;
  private double setpoint;
  private PIDController pidController = new PIDController(0.030, 0, 0);

  /*
   * From what I could gather running simulations of PIDs on the WPILIB docs, The Integral Values can remain at 0,
   * while adjusting the P values to allow the system to reach the setpoint without oscillation, then adjusting the D value CAREFULLY
   * to allow the system to reach the setpoint as quickly as possible without overshooting.
   *
   * Basically alot more tuning is needed for a faster and more percise intake.
   * so... TODO: Tune intake PID to be constistantly fast and percise going to all setpoint values
   */
  // Old PID Values
  // private PIDController pidController = new PIDController(0.42, 0.015, 0);

  // New PID Values
  // private PIDController pidController = new PIDController(0.95, 0.0, 0.0001);

  public WristSetpointCommand(EndEffectorSubsystem wrist, double setpoint) {
    this.wrist = wrist;
    this.setpoint = setpoint;

    // pidController.setSetpoint(setpoint);

    // pidController.setIZone(20);
    addRequirements(wrist);

    pidController.setTolerance(0);
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {

    double speed = pidController.calculate(wrist.getWristEncoder(), setpoint);

    wrist.setWristSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    wrist.setWristSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}

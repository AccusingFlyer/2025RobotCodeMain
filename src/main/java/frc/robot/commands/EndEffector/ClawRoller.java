package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ClawRoller extends Command {
  private EndEffectorSubsystem roller;
  private double volts;

  public ClawRoller(EndEffectorSubsystem roller, double volts) {
    this.roller = roller;
    this.volts = volts;

    // addRequirements(intake); //uncomment this if its being overruled by other commands
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    roller.setPowerVolts(volts);
  }

  @Override
  public void end(boolean interrupted) {
    // if (volts > 0) {
    //     intake.setPowerVolts(0.2);
    // } else {
    //     intake.setPowerVolts(-0.2);
    // }
    // intake.setPivotVolts(volts / 2);
    roller.setPowerVolts(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

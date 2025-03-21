package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class HalfSpitCommand extends Command {
  private final EndEffectorSubsystem wrist;
  private final Timer timer = new Timer();

  private static final double SPIT_OUT_POWER = 0.1;
  private static final double SPIT_OUT_DURATION = 0.25;

  public HalfSpitCommand(EndEffectorSubsystem wrist) {
    this.wrist = wrist;
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    timer.restart();

    wrist.setPowerVolts(SPIT_OUT_POWER);
  }

  @Override
  public void execute() {
    // No-op; just waiting on timer
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(SPIT_OUT_DURATION);
  }

  @Override
  public void end(boolean interrupted) {
    wrist.setPowerVolts(0);
  }
}

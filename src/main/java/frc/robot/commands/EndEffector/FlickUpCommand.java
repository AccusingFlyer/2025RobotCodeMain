package frc.robot.commands.EndEffector;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class FlickUpCommand extends Command {
  private final EndEffectorSubsystem wrist;
  private final Timer timer = new Timer();

  private static final double FLICK_UP_POWER = 0.25;
  private static final double FLICK_UP_DURATION = 0.5;

  public FlickUpCommand(EndEffectorSubsystem wrist) {
    this.wrist = wrist;
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    timer.restart();
    wrist.setNeutralMode(NeutralModeValue.Coast);
    wrist.setWristSpeed(FLICK_UP_POWER);
  }

  @Override
  public void execute() {
    // No-op; just waiting on timer
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(FLICK_UP_DURATION);
  }

  @Override
  public void end(boolean interrupted) {
    wrist.setWristSpeed(0);
    System.out.println("Flick Up Command Ends");
  }
}

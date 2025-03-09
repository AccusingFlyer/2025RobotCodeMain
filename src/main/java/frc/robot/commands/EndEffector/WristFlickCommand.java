package frc.robot.commands.EndEffector;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class WristFlickCommand extends SequentialCommandGroup {

  private static final double FLICK_UP_POWER = 0.25;
  private static final double FLICK_DOWN_POWER = -0.25;
  private static final double FLICK_UP_DURATION = 0.25;
  private static final double FLICK_DOWN_DURATION = 0.1;

  public WristFlickCommand(EndEffectorSubsystem wrist) {
    addCommands(
        new ParallelRaceGroup(new FlickUpCommand(wrist), new WaitCommand(FLICK_UP_DURATION)),
        new ParallelRaceGroup(new FlickDownCommand(wrist), new WaitCommand(FLICK_DOWN_DURATION)),
        new LockWristCommand(wrist));
  }

  public static class FlickUpCommand extends Command {
    private final EndEffectorSubsystem wrist;

    public FlickUpCommand(EndEffectorSubsystem wrist) {
      this.wrist = wrist;
      addRequirements(wrist);
    }

    @Override
    public void initialize() {
      wrist.setNeutralMode(NeutralModeValue.Coast);
      wrist.setWristSpeed(FLICK_UP_POWER);
    }

    @Override
    public void end(boolean interrupted) {
      wrist.setWristSpeed(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }

  public static class FlickDownCommand extends Command {
    private final EndEffectorSubsystem wrist;

    public FlickDownCommand(EndEffectorSubsystem wrist) {
      this.wrist = wrist;
      addRequirements(wrist);
    }

    @Override
    public void initialize() {
      wrist.setWristSpeed(FLICK_DOWN_POWER);
    }

    @Override
    public void end(boolean interrupted) {
      wrist.setWristSpeed(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }

  public static class LockWristCommand extends Command {
    private final EndEffectorSubsystem wrist;

    public LockWristCommand(EndEffectorSubsystem wrist) {
      this.wrist = wrist;
      addRequirements(wrist);
    }

    @Override
    public void initialize() {
      wrist.setWristSpeed(0);
      wrist.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
  }
}

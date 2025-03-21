package frc.robot.commands.EndEffector;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffectorSubsystem;

public class WristFlickCommand extends SequentialCommandGroup {

  private static final double FLICK_UP_POWER = 0.25;
  private static final double FLICK_DOWN_POWER = -0.25;
  private static final double FLICK_UP_DURATION = 0.25;
  private static final double FLICK_DOWN_DURATION = 0.1;

  public WristFlickCommand(EndEffectorSubsystem wrist) {
    addCommands(new FlickUpCommand(wrist), new FlickDownCommand(wrist));
  }

  public static class FlickUpCommand extends Command {
    private final EndEffectorSubsystem wrist;
    private final Timer timer = new Timer();

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

  public static class FlickDownCommand extends Command {
    private final EndEffectorSubsystem wrist;
    private final Timer timer = new Timer();

    public FlickDownCommand(EndEffectorSubsystem wrist) {
      this.wrist = wrist;
      addRequirements(wrist);
    }

    @Override
    public void initialize() {
      timer.restart();
      wrist.setWristSpeed(FLICK_DOWN_POWER);
    }

    @Override
    public void execute() {
      // No-op
    }

    @Override
    public boolean isFinished() {
      return timer.hasElapsed(FLICK_DOWN_DURATION);
    }

    @Override
    public void end(boolean interrupted) {
      wrist.setWristSpeed(0);
      System.out.println("Flick Down Command Ends");
    }
  }

  // public static class LockWristCommand extends Command {
  //   private final EndEffectorSubsystem wrist;

  //   public LockWristCommand(EndEffectorSubsystem wrist) {
  //     this.wrist = wrist;
  //     addRequirements(wrist);
  //   }

  //   @Override
  //   public void initialize() {
  //     wrist.setWristSpeed(0);
  //     wrist.setNeutralMode(NeutralModeValue.Brake);
  //   }

  //   @Override
  //   public boolean isFinished() {
  //     return true;

  //   }
  // }
}

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LevelFourRightAlign extends Command {
  private PIDController xController, yController, rotController;
  private Timer dontSeeTagTimer, stopTimer;
  private DrivetrainSubsystem drive;

  private boolean hasStartedScoring = false;

  public LevelFourRightAlign(DrivetrainSubsystem drive) {
    xController = new PIDController(2.5, 0, 0); // Vertical movement
    yController = new PIDController(2.5, 0, 0); // Horitontal movement
    rotController = new PIDController(0.025, 0, 0); // Rotation

    this.drive = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(-16.5);
    rotController.setTolerance(0.5);

    xController.setSetpoint(-1.03);
    xController.setTolerance(0.8);

    yController.setSetpoint(-0.46);
    yController.setTolerance(0.8);

    hasStartedScoring = false;
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-c")) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-c");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drive.drive(-xSpeed, -ySpeed, rotValue, false, true);

      if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
        stopTimer.reset();
      }

      //   if (stopTimer.hasElapsed(2.0) && !hasStartedScoring) {
      //     hasStartedScoring = true;
      //     autoScore();
      //   }

    } else {
      drive.drive(0, 0, 0, false, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, false, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag
    // in the camera
    return this.dontSeeTagTimer.hasElapsed(1) || stopTimer.hasElapsed(0.3);
  }

  //   private void autoScore() {
  //     new SequentialCommandGroup(
  //             // Step 1: Start the claw rollers immediately
  //             new ClawRoller(roller, 5),

  //             // Step 2: Wait 0.3 seconds before starting the flick
  //             new ParallelRaceGroup(
  //                 new ClawRoller(roller, -3.5), // Rollers keep rolling
  //                 new SequentialCommandGroup(
  //                     new WaitCommand(0.3), // Delay for 0.3 seconds
  //                     new WristFlickCommand(roller) // Then flick while rollers are still rolling
  //                     )),

  //         .schedule();
  //   }
}

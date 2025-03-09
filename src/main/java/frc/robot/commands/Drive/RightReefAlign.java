package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RightReefAlign extends Command {
  private PIDController xController, yController, rotController;
  private Timer dontSeeTagTimer, stopTimer;
  private DrivetrainSubsystem drive;

  public RightReefAlign(DrivetrainSubsystem drive) {
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

    xController.setSetpoint(-0.89);
    xController.setTolerance(0.8);

    yController.setSetpoint(-0.455);
    yController.setTolerance(0.8);
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
}

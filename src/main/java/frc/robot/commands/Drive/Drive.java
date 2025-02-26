package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class Drive extends Command {
  private final DrivetrainSubsystem s_Swerve;
  private final DoubleSupplier translationSup;
  private final DoubleSupplier strafeSup;
  private final DoubleSupplier rotationSup;
  private final boolean robotCentricSup;

  private final Timer previousRotationalInputTimer;
  private final PIDController chassisRotationController;
  private Rotation2d rotationMaintenanceSetpoint;

  public Drive(
      DrivetrainSubsystem s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      boolean robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;

    this.previousRotationalInputTimer = new Timer();
    this.chassisRotationController = new PIDController(0.01, 0, 0); // Tune PID values
    this.chassisRotationController.enableContinuousInput(0, Math.toRadians(360));
  }

  @Override
  public void initialize() {
    previousRotationalInputTimer.start();
    rotationMaintenanceSetpoint = s_Swerve.getGyroscopeRotation();
  }

  @Override
  public void execute() {
    /* Get Values, Apply Deadband */
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), 0.1);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.1);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.1);

    /* Reset rotation timer if there is rotation input */
    if (rotationVal != 0) {
      previousRotationalInputTimer.reset();
    }

    final double rotationSpeedOmega;
    /* No rotation input for 0.5 seconds, maintain current rotation */
    if (previousRotationalInputTimer.hasElapsed(0.5)) {
      rotationSpeedOmega =
          chassisRotationController.calculate(
              s_Swerve.getGyroDouble(), rotationMaintenanceSetpoint.getRadians());
    } else {
      rotationSpeedOmega = rotationVal * Constants.DriveConstants.maxAngularVelocity;
      rotationMaintenanceSetpoint = s_Swerve.getPose().getRotation();
    }

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.DriveConstants.maxSpeed),
        rotationSpeedOmega,
        !robotCentricSup,
        true);
  }
}

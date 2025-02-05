package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.settings.Constants;
import frc.robot.settings.Conversions;
import frc.robot.settings.SwerveModuleConstants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANcoder angleEncoder;

  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(
          Constants.DriveConstants.driveKS,
          Constants.DriveConstants.driveKV,
          Constants.DriveConstants.driveKA);

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    angleEncoder.getConfigurator().apply(Constants.CTREConfigs.getSteerEncoderConfig());

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
    mAngleMotor.getConfigurator().apply(Constants.CTREConfigs.getSteerMotorConfig());
    resetToAbsolute();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
    mDriveMotor.getConfigurator().apply(Constants.CTREConfigs.getDriveMotorConfig());
    mDriveMotor.getConfigurator().setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.DriveConstants.maxSpeed;
      mDriveMotor.setControl(driveDutyCycle);
    } else {
      driveVelocity.Velocity =
          Conversions.MPSToRPS(
              desiredState.speedMetersPerSecond, Constants.DriveConstants.wheelCircumference);
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      mDriveMotor.setControl(driveVelocity);
    }
  }

  public double getSpeedMetersPerSecond() {
    return (mDriveMotor.getVelocity().getValueAsDouble() * (0.092 * Math.PI));
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromRotations(
        MathUtil.inputModulus(mAngleMotor.getPosition().getValueAsDouble(), -0.5, 0.5));
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.RPSToMPS(
            mDriveMotor.getVelocity().getValueAsDouble(),
            Constants.DriveConstants.wheelCircumference),
        Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getSpeedMetersPerSecond(), getRotation());
  }

  public void stop() {
    mDriveMotor.set(0);
    mAngleMotor.set(0);
    ;
  }
}

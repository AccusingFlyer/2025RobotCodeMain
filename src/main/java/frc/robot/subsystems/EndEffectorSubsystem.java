package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class EndEffectorSubsystem extends SubsystemBase {
  private TalonFX powerMotor = new TalonFX(Constants.EndEffectorConstants.EFFECTOR_MOTOR_1_ID);
  private TalonFX wristMotor = new TalonFX(Constants.EndEffectorConstants.EFFECTOR_MOTOR_2_ID);

  public EndEffectorSubsystem() {
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    powerMotor.setNeutralMode(NeutralModeValue.Brake);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.6; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 1; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 1; // An acceleration of 1 rps/s requires 0.01 V output .01
    slot0Configs.kP = 1; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 160 rps/s (0.5 seconds) 60
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    wristMotor.getConfigurator().apply(talonFXConfigs);

    var talonFXConfiguration1 = new TalonFXConfiguration();

    talonFXConfiguration1.CurrentLimits.StatorCurrentLimit = 130;
    talonFXConfiguration1.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfiguration1.CurrentLimits.SupplyCurrentLimit = 75;
    talonFXConfiguration1.CurrentLimits.SupplyCurrentLimitEnable = true;

    powerMotor.getConfigurator().apply(talonFXConfiguration1);
  }

  public void setPowerVolts(double volts) {
    powerMotor.setVoltage(volts);
  }

  public double getPowerVelocity() {
    return powerMotor.getVelocity().getValueAsDouble();
  }

  public void zeroWristEncoder() {
    wristMotor.setPosition(0);
  }

  public double getWristEncoder() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  public void setWristVolts(double volts) {
    wristMotor.setVoltage(volts);
  }

  public void moveToSetpoint(double setpoint) {
    final MotionMagicVoltage request = new MotionMagicVoltage(0);

    wristMotor.setControl(request.withPosition(setpoint));
  }

  public void setWristSpeed(double speed) {
    wristMotor.set(speed);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Intake Limit Switch", getLimitSwitch());
    SmartDashboard.putNumber("Wrist Encoder", getWristEncoder());
    SmartDashboard.putNumber("Power Motor Velocity", getPowerVelocity());
  }
}

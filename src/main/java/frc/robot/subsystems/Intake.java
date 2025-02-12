package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Intake extends SubsystemBase {
  private TalonFX pivotMotor1 = new TalonFX(Constants.IntakeConstants.PIVOT_MOTOR_1_ID);
  private TalonFX pivotMotor2 = new TalonFX(Constants.IntakeConstants.PIVOT_MOTOR_2_ID);
  private TalonFX pivotMotor3 = new TalonFX(Constants.IntakeConstants.PIVOT_MOTOR_3_ID);

  private TalonFX intakeSpinMotor = new TalonFX(Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID);

  public Intake() {
    pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor2.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor3.setNeutralMode(NeutralModeValue.Brake);
    intakeSpinMotor.setNeutralMode(NeutralModeValue.Brake);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.09; // An acceleration of 1 rps/s requires 0.01 V output .01
    slot0Configs.kP = 0.15; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        80; // Target acceleration of 160 rps/s (0.5 seconds) 60
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    pivotMotor1.getConfigurator().apply(talonFXConfigs);
    pivotMotor2.getConfigurator().apply(talonFXConfigs);
    pivotMotor3.getConfigurator().apply(talonFXConfigs);

    pivotMotor2.setControl(new Follower(Constants.IntakeConstants.PIVOT_MOTOR_1_ID, false));
    pivotMotor3.setControl(new Follower(Constants.IntakeConstants.PIVOT_MOTOR_1_ID, false));

    var talonFXConfiguration1 = new TalonFXConfiguration();

    talonFXConfiguration1.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfiguration1.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfiguration1.CurrentLimits.SupplyCurrentLimit = 22;
    talonFXConfiguration1.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeSpinMotor.getConfigurator().apply(talonFXConfiguration1);
  }

  public void zeroPivot1Encoder() {
    pivotMotor1.setPosition(0);
  }

  public void zeroPivot2Encoder() {
    pivotMotor2.setPosition(0);
  }

  public void zeroPivot3Encoder() {
    pivotMotor3.setPosition(0);
  }

  public double getPivot1Encoder() {
    return pivotMotor1.getPosition().getValueAsDouble();
  }

  public double getPivot2Encoder() {
    return pivotMotor2.getPosition().getValueAsDouble();
  }

  public double getPivot3Encoder() {
    return pivotMotor3.getPosition().getValueAsDouble();
  }

  public void setPivotVolts(double volts) {
    pivotMotor1.setVoltage(volts);
  }

  public void moveToSetpoint(double setpoint) {
    final MotionMagicVoltage request = new MotionMagicVoltage(0);

    pivotMotor1.setControl(request.withPosition(setpoint));
  }

  public void setPivotSpeed(double speed) {
    pivotMotor1.set(speed);
  }

  public void setPowerVolts(double volts) {
    intakeSpinMotor.setVoltage(volts);
  }

  public double getPowerVelocity() {
    return intakeSpinMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Motor 1", getPivot1Encoder());
    SmartDashboard.putNumber("Pivot Motor 2", getPivot2Encoder());
    SmartDashboard.putNumber("Pivot Motor 3", getPivot3Encoder());
  }
}

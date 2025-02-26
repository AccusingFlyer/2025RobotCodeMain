package frc.robot.subsystems;

import static frc.robot.settings.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor1;
  private TalonFX elevatorMotor2;
  private TalonFXConfiguration eleMotorConfig;
  private double zeroPoint;
  MotorLogger motorLogger1;
  MotorLogger motorLogger2;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor1 = new TalonFX(ELEVATOR_MOTOR_1_ID);
    elevatorMotor2 = new TalonFX(ELEVATOR_MOTOR_2_ID);

    var talonFXConfigs = new TalonFXConfiguration();

    elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);

    // Primary PID - Velocity Control
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = 0.1; // Proportional gain for velocity control
    slot0Configs.kI = 0.0; // Integral gain
    slot0Configs.kD = 0.01; // Derivative gain
    slot0Configs.kS = 0.30; // Static friction gain
    slot0Configs.kG = 0.60; // Gravity gain
    slot0Configs.kV = 1; // Velocity feedforward gain
    slot0Configs.kA = 0.01; // Acceleration feedforward gain

    // Set elevatorMotor2 to follow elevatorMotor1
    elevatorMotor2.setControl(new Follower(ELEVATOR_MOTOR_1_ID, true));

    talonFXConfigs.Feedback.SensorToMechanismRatio = 4.375;

    elevatorMotor2.getConfigurator().apply(talonFXConfigs);
    elevatorMotor1.getConfigurator().apply(talonFXConfigs);

    motorLogger1 = new MotorLogger("/elevator/motor1");
    motorLogger2 = new MotorLogger("/elevator/motor2");
  }

  private void logMotors() {
    motorLogger1.log(elevatorMotor1);
    motorLogger2.log(elevatorMotor2);
  }

  public double getVelocityLeft() {
    return -elevatorMotor1.getVelocity().getValueAsDouble();
  }

  public double getVelocityRight() {
    return elevatorMotor2.getVelocity().getValueAsDouble();
  }

  /**
   * Sets the elevator to a target velocity.
   *
   * @param velocity double that controls the target velocity in rps
   */
  public void setElevatorVelocity(double velocity) {
    VelocityVoltage velocityRequest = new VelocityVoltage(velocity);

    // Set motors with velocity control
    elevatorMotor1.setControl(velocityRequest.withVelocity(-velocity));
    elevatorMotor2.setControl(velocityRequest.withVelocity(velocity));
  }

  public boolean isElevatorAtVelocity() {
    return Math.abs(elevatorMotor1.getClosedLoopError().getValueAsDouble()) < ELEVATOR_THRESHOLD;
  }

  public void setMotors(double speed1, double speed2) {
    elevatorMotor1.set(-speed1);
    elevatorMotor2.set(speed2);
  }

  public void stopElevator() {
    elevatorMotor1.set(0);
    elevatorMotor2.set(0);
  }

  public void zeroMotorEncoders() {
    elevatorMotor1.setPosition(0);
    elevatorMotor2.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotors();

    SmartDashboard.putNumber("Left Elevator Velocity", getVelocityLeft());
    SmartDashboard.putNumber("Right Elevator Velocity", getVelocityRight());
  }
}

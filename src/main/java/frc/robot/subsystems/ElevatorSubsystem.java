// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
  DigitalInput elevatorHallEffect1;
  DigitalInput elevatorHallEffect2;
  MotorLogger motorLogger1;
  MotorLogger motorLogger2;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor1 = new TalonFX(ELEVATOR_MOTOR_1_ID);
    elevatorMotor2 = new TalonFX(ELEVATOR_MOTOR_2_ID);

    var talonFXConfigs = new TalonFXConfiguration();

    elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor2.setNeutralMode(NeutralModeValue.Brake);

    // var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = 0.30; // Add 0.25 V output to overcome static friction
    // slot0Configs.kG = 0.60;
    // slot0Configs.kV = 1; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kA = 1; // An acceleration of 1 rps/s requires 0.01 V output .01
    // slot0Configs.kP = 1.8; // A position error of 2.5 rotations results in 12 V output
    // slot0Configs.kI = 0.0; // no output for integrated error
    // slot0Configs.kD = 1.6; // A velocity error of 1 rps results in 0.1 V output

    // Primary PID - Height Control
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = 1.8;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 1.6;
    slot0Configs.kS = 0.30;
    slot0Configs.kG = 0.60;
    slot0Configs.kV = 1;
    slot0Configs.kA = 1;

    // Auxiliary PID - Synchronization Control
    var slot1Configs = talonFXConfigs.Slot1;
    slot1Configs.kP = 1.0; // Lower value to keep sides synced without overcorrecting
    slot1Configs.kI = 0.0;
    slot1Configs.kD = 0.5;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 5.1; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        7.3; // Target acceleration of 160 rps/s (0.5 seconds) 60
    motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // .withMotionMagic(new MotionMagicConfigs()
    // .withMotionMagicAcceleration(2531)
    // .withMotionMagicCruiseVelocity(2531)
    // .withMotionMagicJerk(2531));

    // Set elevatorMotor2 to follow elevatorMotor1, with auxiliary control
    elevatorMotor2.setControl(new Follower(ELEVATOR_MOTOR_1_ID, true));

    talonFXConfigs.Feedback.SensorToMechanismRatio = 4.375;

    elevatorMotor2.getConfigurator().apply(talonFXConfigs);
    elevatorMotor1.getConfigurator().apply(talonFXConfigs);

    motorLogger1 = new MotorLogger("/elevator/motor1");
    motorLogger2 = new MotorLogger("/elevator/motor2");

    // elevatorHallEffect1 = new DigitalInput(0);
    // elevatorHallEffect2 = new DigitalInput(1);
  }

  private void logMotors() {
    motorLogger1.log(elevatorMotor1);
    motorLogger2.log(elevatorMotor2);
  }

  public double getEncoderLeft() {
    return -elevatorMotor1.getPosition().getValueAsDouble();
  }

  public double getEncoderRight() {
    return elevatorMotor2.getPosition().getValueAsDouble();
  }

  /**
   * Sets the elevator to a position relative to the 0 set by createZero.
   *
   * @param height double that controls how many millimeters from the distance sensor
   */
  public void setElevatorPosition(double height) {

    double avgPosition = (-getEncoderLeft() + getEncoderRight()) / 2.0;
    double deviation = getEncoderRight() + getEncoderLeft();

    // Motion Magic position control
    MotionMagicVoltage primaryRequest = new MotionMagicVoltage(height);

    // Apply correction for synchronization
    double correctionOutput = deviation * 0.1; // Tune this value

    // Apply primary PID for height control
    // double primaryOutput = request.withPosition(height).getReference();

    // Set motors with primary control and synchronization correction
    elevatorMotor1.setControl(primaryRequest.withPosition(-height - correctionOutput));
    elevatorMotor2.setControl(primaryRequest.withPosition(height + correctionOutput));
  }

  // public void setElevatorPosition(ElevatorStates height) {
  //   switch (height) {
  //     case Reef1:
  //       setElevatorPosition(REEF_LEVEL_1_MILLIMETERS);
  //       break;
  //     case Reef2:
  //       setElevatorPosition(REEF_LEVEL_2_MILLIMETERS);
  //       break;
  //     case Reef3:
  //       setElevatorPosition(REEF_LEVEL_3_MILLIMETERS);
  //       break;
  //     case Reef4:
  //       setElevatorPosition(REEF_LEVEL_4_MILLIMETERS);
  //       break;
  //     case HumanPlayer:
  //       setElevatorPosition(HUMAN_PLAYER_STATION_MILLIMETERS);
  //       break;
  //   }
  // }

  public boolean isElevatorAtPose() {
    return elevatorMotor1.getClosedLoopError().getValueAsDouble() < ELEVATOR_THRESHOLD;
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
    // if (elevatorHallEffect1.get()) {
    //   elevatorMotor1.setPosition(0);
    // }

    // if (elevatorHallEffect2.get()) {
    //   elevatorMotor2.setPosition(0);
    // }

    SmartDashboard.putNumber("Left Elevator", getEncoderLeft());
    SmartDashboard.putNumber("Right Elevator", getEncoderRight());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;
import frc.robot.settings.ElevatorStates;

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

    eleMotorConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(1).withKS(0).withKA(0).withKV(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(100)
                    .withSupplyCurrentLimitEnable(true));

    // .withMotionMagic(new MotionMagicConfigs()
    // .withMotionMagicAcceleration(2531)
    // .withMotionMagicCruiseVelocity(2531)
    // .withMotionMagicJerk(2531));
    elevatorMotor1.getConfigurator().apply(eleMotorConfig);
    elevatorMotor2.getConfigurator().apply(eleMotorConfig);
    elevatorMotor2.setControl(new Follower(ELEVATOR_MOTOR_1_ID, true));

    motorLogger1 = new MotorLogger("/elevator/motor1");
    motorLogger2 = new MotorLogger("/elevator/motor2");

    elevatorHallEffect1 = new DigitalInput(0);
    elevatorHallEffect2 = new DigitalInput(1);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotors();
    if (elevatorHallEffect1.get()) {
      elevatorMotor1.setPosition(0);
    }

    if (elevatorHallEffect2.get()) {
      elevatorMotor2.setPosition(0);
    }

    SmartDashboard.putNumber("Left Elevator", getEncoderLeft());
    SmartDashboard.putNumber("Right Elevator", getEncoderRight());
  }
  /**
   * Sets the elevator to a position relative to the 0 set by createZero.
   *
   * @param height double that controls how many millimeters from the distance sensor
   */
  public void setElevatorPosition(double height) {

    final MotionMagicVoltage request = new MotionMagicVoltage(0);
    elevatorMotor1.setControl(request.withPosition(height));
  }

  public void setElevatorPosition(ElevatorStates height) {
    switch (height) {
      case Reef1:
        setElevatorPosition(REEF_LEVEL_1_MILLIMETERS);
        break;
      case Reef2:
        setElevatorPosition(REEF_LEVEL_2_MILLIMETERS);
        break;
      case Reef3:
        setElevatorPosition(REEF_LEVEL_3_MILLIMETERS);
        break;
      case Reef4:
        setElevatorPosition(REEF_LEVEL_4_MILLIMETERS);
        break;
      case HumanPlayer:
        setElevatorPosition(HUMAN_PLAYER_STATION_MILLIMETERS);
        break;
    }
  }

  public boolean isElevatorAtPose() {
    return elevatorMotor1.getClosedLoopError().getValueAsDouble() < ELEVATOR_THRESHOLD;
  }

  public void stopElevator() {
    elevatorMotor1.set(0);
  }
}

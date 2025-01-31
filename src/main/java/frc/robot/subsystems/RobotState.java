package frc.robot.subsystems;

import frc.robot.settings.ElevatorStates;

public class RobotState {
  private static RobotState instance;
  public boolean LimelightsUpdated;
  public ElevatorStates deliveringCoralHeight;
  public double odometerOrientation;

  private RobotState() {}

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
}

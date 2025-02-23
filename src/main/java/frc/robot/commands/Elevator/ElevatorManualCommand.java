package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManualCommand extends Command {

  private ElevatorSubsystem elevator;

  public ElevatorManualCommand(ElevatorSubsystem elevator) {
    this.elevator = elevator;

    addRequirements(elevator);
  }

  @Override
  public void execute() {
    double speed = -MathUtil.applyDeadband(RobotContainer.operatorControllerXbox.getRightY(), 0.1);

    elevator.setMotors(speed, speed);
  }
}

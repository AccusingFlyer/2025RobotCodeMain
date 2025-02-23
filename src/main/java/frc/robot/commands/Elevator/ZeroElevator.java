package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevator extends Command {

  private ElevatorSubsystem elevator;

  public ZeroElevator(ElevatorSubsystem elevator) {
    this.elevator = elevator;

    addRequirements(elevator);
  }

  @Override
  public void execute() {
    double speed = -MathUtil.applyDeadband(RobotContainer.operatorControllerXbox.getLeftY(), 0.1);

    elevator.setMotors(speed, speed);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setMotors(0, 0);
    elevator.zeroMotorEncoders();
  }
}

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSetpointCommand extends Command {
  private final ElevatorSubsystem m_subsystem;
  private final double m_goal;

  public ElevatorSetpointCommand(ElevatorSubsystem subsystem, double goal) {
    m_subsystem = subsystem;
    m_goal = goal;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.setElevatorGoal(m_goal);
  }

  @Override
  public void execute() {
    m_subsystem.updateElevator();
  }

  @Override
  public boolean isFinished() {
    // Command is finished when the elevator reaches its goal
    return m_subsystem.getEncoderDistance() >= m_goal - 0.1
        && m_subsystem.getEncoderDistance() <= m_goal + 0.1;
  }

  @Override
  public void end(boolean interrupted) {
    // Optionally stop the motor when the command ends
    m_subsystem.updateElevator();
  }
}

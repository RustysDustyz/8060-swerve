package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
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
    double currentHeight = m_subsystem.getElevatorPositionMeters();
    return Math.abs(currentHeight - m_goal) < 0.05; // Allow small tolerance
  }

  @Override
  public void end(boolean interrupted) {
    // Stop motor when command ends
    m_subsystem.stopElevator();
  }
}
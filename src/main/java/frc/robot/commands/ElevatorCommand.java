package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private ElevatorSubsystem s_Elevator;
    private double speed;

    public ElevatorCommand(ElevatorSubsystem s_Elevator, DoubleSupplier speed) {
        this.s_Elevator = s_Elevator;
        this.speed = speed.getAsDouble();
        addRequirements(s_Elevator);
    }

    @Override
    public void initialize() {
        s_Elevator.moveElevator(speed);
    }


    @Override
    public void end(boolean interrupted) { }
    

}

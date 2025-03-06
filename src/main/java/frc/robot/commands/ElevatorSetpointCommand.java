package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ElevatorSetpointCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final int heightIndex;
    private final int angleIndex;

    public ElevatorSetpointCommand(ElevatorSubsystem elevator, int heightIndex, int angleIndex) {
        this.elevator = elevator;
        this.heightIndex = heightIndex;
        this.angleIndex = angleIndex;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        //System.out.println("Starting setpoint command: Elevator " + heightIndex + ", Wrist " + angleIndex);
    }

    @Override
    public void execute() {
        elevator.moveToHeight(heightIndex);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtHeight(heightIndex);// && wrist.isAtAngle();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            //System.out.println("Setpoint command interrupted.");
            elevator.stop();
        }
        //System.out.println("Setpoint command completed.");
    }
}

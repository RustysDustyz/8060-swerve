package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ElevatorSetpointCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final WristSubsystem wrist;
    private final int heightIndex;
    private final int angleIndex;

    public ElevatorSetpointCommand(ElevatorSubsystem elevator, WristSubsystem wrist, int heightIndex, int angleIndex) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.heightIndex = heightIndex;
        this.angleIndex = angleIndex;
        addRequirements(elevator, wrist);
    }

    @Override
    public void initialize() {
        if (!CommandScheduler.getInstance().isScheduled(elevator.getDefaultCommand())) {
            // new Thread(() -> {
            //     elevator.moveToHeight(targetIndex); // move to angle too 
            // }).start();
            elevator.moveToHeight(heightIndex);
            wrist.moveToAngle(angleIndex);
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtHeight(heightIndex) && wrist.isAtAngle(); // Both must be in tolerance
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.stop();
            wrist.stop();
        }
    }
}

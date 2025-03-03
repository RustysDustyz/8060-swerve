package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IOSubsystem;

public class IOCommand extends Command {
    private IOSubsystem subsystem;
    private DoubleSupplier speed;

    public IOCommand(IOSubsystem subsystem, DoubleSupplier speed) {
        this.subsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.set(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { }
    

}

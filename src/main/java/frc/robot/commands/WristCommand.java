package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends Command {
    private WristSubsystem subsystem;
    private DoubleSupplier clawSpeed;
    private DoubleSupplier intakeSpeed;

    public WristCommand(WristSubsystem subsystem, DoubleSupplier clawSpeed, DoubleSupplier intakeSpeed) {
        this.subsystem = subsystem;
        this.clawSpeed = clawSpeed;
        this.intakeSpeed = intakeSpeed;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setClaw(clawSpeed.getAsDouble());
        subsystem.setIntake(intakeSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { }
    

}

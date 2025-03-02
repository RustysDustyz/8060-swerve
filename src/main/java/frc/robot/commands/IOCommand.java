package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class IOCommand extends Command {
    private WristSubsystem s_Wrist;
    private double speed;

    public IOCommand(WristSubsystem s_Wrist, DoubleSupplier speed) {
        this.s_Wrist = s_Wrist;
        this.speed = speed.getAsDouble();
        addRequirements(s_Wrist);
    }

    @Override
    public void initialize() {
        s_Wrist.moveWrist(speed);
        
    }


    @Override
    public void end(boolean interrupted) { }
    

}

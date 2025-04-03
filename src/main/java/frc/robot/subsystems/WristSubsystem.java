package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class WristSubsystem extends IOSubsystem {
    private final SparkMax wristMotor;
    private final SparkMax intakeMotor;
    private final RelativeEncoder wristEncoder;

    // Error margin (tolerance) in motor rotations
    private static final double ERROR_MARGIN = 0.5; // Adjust as needed

    private boolean manualControlActive = false; // Flag for preventing periodic interference

    private double targetPosition = 0; // Stores the latest target position

    public WristSubsystem() {
        wristMotor = new SparkMax(ElevatorConstants.wristMotorID, MotorType.kBrushless);
        intakeMotor = new SparkMax(ElevatorConstants.intakeMotorID, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        
        wristEncoder.setPosition(0); // Reset encoder at startup
    }

    public boolean isAtAngle() {
        return Math.abs(wristEncoder.getPosition() - targetPosition) < ERROR_MARGIN;
    }

    public double getAngle() {
        return (wristEncoder.getPosition()* 360)/(Math.PI*2);
    }

    public void resetDistance(){
        wristEncoder.setPosition(0);
    }
    
    @Override
    public void stop() {
        wristMotor.set(0);
    }

    public void setClaw(double speed) {
        //System.out.println(speed);
        if (speed < 0) {
            wristMotor.set(0.12*speed);
        } else if (speed > 0){
            wristMotor.set(0.18*speed);
        } else {
            wristMotor.set(0.01);
        }
        SmartDashboard.putNumber("angle", getAngle());
    }

    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }
    
    @Override
    public void periodic() {
    
    }
}
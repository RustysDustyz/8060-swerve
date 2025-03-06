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
    // Wrist positions in motor rotations (adjust as needed)
    private static final double[] ANGLES = {-30.0, -15.0, 0.0, 15.0, 30.0}; // Example values in motor rotations

    // Error margin (tolerance) in motor rotations
    private static final double ERROR_MARGIN = 1.0; // Adjust as needed
    private static final double MOTOR_SPEED = 0.15; // Adjust based on testing

    private boolean manualControlActive = false; // Flag for preventing periodic interference

    private double targetPosition = 0; // Stores the latest target position

    public WristSubsystem() {
        wristMotor = new SparkMax(ElevatorConstants.wristMotorID, MotorType.kBrushless);
        intakeMotor = new SparkMax(ElevatorConstants.intakeMotorID, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        
        wristEncoder.setPosition(0); // Reset encoder at startup
    }

    public void moveToAngle (int angleIndex) {
        manualControlActive = true;
        double targetPosition = ANGLES[angleIndex];

        while (Math.abs(getAngle() - targetPosition) > ERROR_MARGIN) {
            if (getAngle() < targetPosition) {
                set(MOTOR_SPEED);
            } else {
                set(-MOTOR_SPEED);
            }
        }
        manualControlActive = false;
        stop();
    }

    public boolean isAtAngle() {
        return Math.abs(wristEncoder.getPosition() - targetPosition) < ERROR_MARGIN;
    }

    public double getAngle() {
        return (wristEncoder.getPosition()* 360)/(Math.PI*2);
    }
    
    @Override
    public void stop() {
        wristMotor.set(0);
    }

    public void setClaw(double speed) {
        if (speed != 0) {
            manualControlActive = true;
        } else {
            manualControlActive = false;
        }
        
        SmartDashboard.putNumber("angle", getAngle());
        wristMotor.set(0.15*speed);
    }

    public void setIntake(double speed) {
        intakeMotor.set(speed);
    }
    
    @Override
    public void periodic() {
        System.out.println(manualControlActive);
        if (!manualControlActive) {
            System.out.println("test");
            wristMotor.set(0.1);
        }
    }
}
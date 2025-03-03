package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;

public class WristSubsystem extends IOSubsystem {
    private final SparkMax wristMotor;
    private final RelativeEncoder wristEncoder;
    private final PIDController pidController;

    // Wrist positions in motor rotations (adjust as needed)
    private static final double[] ANGLES = {-30.0, -15.0, 0.0, 15.0, 30.0}; // Example values in motor rotations

    // Error margin (tolerance) in motor rotations
    private static final double ERROR_MARGIN = 1.0; // Adjust as needed
    private static final double MOTOR_SPEED = 0.15; // Adjust based on testing

    // PID gains (tune based on testing)
    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private double targetPosition = 0; // Stores the latest target position

    public WristSubsystem() {
        wristMotor = new SparkMax(ElevatorConstants.wristMotorID, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        pidController = new PIDController(kP, kI, kD);
        
        wristEncoder.setPosition(0); // Reset encoder at startup
    }

    public void setAngle(int angleIndex) {
        if (angleIndex < 0 || angleIndex >= ANGLES.length) return;
        targetPosition = ANGLES[angleIndex];
        double output = pidController.calculate(wristEncoder.getPosition(), targetPosition);
        wristMotor.set(output);
    }

    public void moveToAngle (int angleIndex) {
        double targetPosition = ANGLES[angleIndex];

        while (Math.abs(getAngle() - targetPosition) > ERROR_MARGIN) {
            if (getAngle() < targetPosition) {
                set(MOTOR_SPEED);
            } else {
                set(-MOTOR_SPEED);
            }
        }

        stop();
    }

    public boolean isAtAngle() {
        return Math.abs(wristEncoder.getPosition() - targetPosition) < ERROR_MARGIN;
    }

    public double getAngle() {
        return (0.000767 * wristEncoder.getPosition()* 360)/(Math.PI*2);
    }
    
    @Override
    public void stop() {
        wristMotor.set(0);
    }

    @Override
    public void set(double speed) {
        wristMotor.set(speed);
    }
}
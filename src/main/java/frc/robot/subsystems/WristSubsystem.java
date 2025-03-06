package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

public class WristSubsystem extends IOSubsystem {
    private final SparkMax wristMotor;
    private final SparkMax intakeMotor;
    private final RelativeEncoder wristEncoder;
    // Wrist positions in motor rotations (adjust as needed)
    private static final double[] ANGLES = {0, -692.13, -724.87, -724.87, 30.0}; // Example values in motor rotations

    // Error margin (tolerance) in motor rotations
    private static final double ERROR_MARGIN = 0.5; // Adjust as needed
    private static final double MOTOR_SPEED = 0.1; // Adjust based on testing

    private boolean manualControlActive = false; // Flag for preventing periodic interference

    private double targetPosition = 0; // Stores the latest target position

    public WristSubsystem() {
        wristMotor = new SparkMax(ElevatorConstants.wristMotorID, MotorType.kBrushless);
        intakeMotor = new SparkMax(ElevatorConstants.intakeMotorID, MotorType.kBrushless);
        wristEncoder = wristMotor.getEncoder();
        
        wristEncoder.setPosition(0); // Reset encoder at startup
    }

    public void moveToAngle(int angleIndex) {
        manualControlActive = true;
        targetPosition = ANGLES[angleIndex]; 
        System.out.println(targetPosition);

    
        double error = getAngle() - targetPosition;
        // Use simple proportional control to move the wrist
        double speed = Math.copySign(MOTOR_SPEED, -error);
        
        if (Math.abs(error) > ERROR_MARGIN) {
            wristMotor.set(speed);
            System.out.println(speed);

            
        } else {
            stop();
            manualControlActive = false;
        }
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
        //System.out.println(speed);
        if (speed < 0) {
            manualControlActive = true;
            wristMotor.set(0.12*speed);
        } else if (speed > 0){
            manualControlActive = true;
            wristMotor.set(0.18*speed);
        } else {
            wristMotor.set(0.01);
            manualControlActive = false;
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
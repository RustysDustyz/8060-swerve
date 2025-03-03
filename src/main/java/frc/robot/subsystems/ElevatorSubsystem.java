package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends IOSubsystem {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final Encoder encoder;

    // Heights in meters (adjust based on testing)
    private static final double[] HEIGHTS = {
      0.0,  // Ground level
      0.18,  // First level
      0.39,  // Second level
      0.63,  // Third level
      0.6,  // Fourth level
    };

    private static final double ERROR_MARGIN = 0.02; // 2 cm tolerance
    private static final double MOTOR_SPEED = 0.15; // Adjust based on testing
    private static final double MAX_HEIGHT = 0.73;

    public ElevatorSubsystem() {
        leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);
        encoder = new Encoder(ElevatorConstants.encoderChannelA, ElevatorConstants.encoderChannelB);
        
        encoder.reset();
    }
    
    public void moveToHeight(int heightIndex) {
        double targetPosition = HEIGHTS[heightIndex];

        while (Math.abs(getHeight() - targetPosition) > ERROR_MARGIN) {
            if (getHeight() < targetPosition) {
                leftMotor.set(MOTOR_SPEED);
                rightMotor.set(-MOTOR_SPEED); // Reverse one motor if needed
            } else {
                leftMotor.set(-MOTOR_SPEED);
                rightMotor.set(MOTOR_SPEED);
            }
        }

        stop(); // Stop motors when within margin
        System.out.println("stop");
    }

    public double getHeight() {
        return encoder.getRaw() * 0.0000194918; // Convert encoder counts to meters
    }

    public boolean isAtHeight(int heightIndex) {
      return Math.abs(getHeight() - HEIGHTS[heightIndex]) < ERROR_MARGIN;
    }

    @Override
    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    @Override
    public void set(double speed) {
        double currentHeight = getHeight();
        
        // Prevent moving up if at or above max height
        if (speed < 0 && currentHeight < 0) {
            stop();
            System.out.println("mined");
            return;
        }

        // Prevent moving down if at or below ground level
        if (speed > 0 && currentHeight > MAX_HEIGHT) {
            stop();
            System.out.println("mined");
            return;
        }
        

          SmartDashboard.putNumber("height", getHeight());
        

        leftMotor.set(speed);
        rightMotor.set(-speed);
        if (speed != 0) {
            System.out.println(speed);
        }
    }

}

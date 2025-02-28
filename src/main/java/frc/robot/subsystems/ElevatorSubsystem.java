package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final Encoder encoder;
    private final PIDController pidController;

    // Heights in meters (adjust based on testing)
    private static final double[] HEIGHTS = {
      0.0,       // Ground level
      0.1,       // First level
      0.3,       // Second level
      0.6,       // Third level
      0.9,       // Fourth level
      1.2        // Fifth level
    };

    // Error margin (tolerance) in meters
    private static final double ERROR_MARGIN = 0.01; // 1 cm tolerance

    // PID gains (tune based on testing)
    private static final double kP = 0.01;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public ElevatorSubsystem() {
      leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
      rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);
      encoder = new Encoder(ElevatorConstants.encoderChannelA, ElevatorConstants.encoderChannelB);

      pidController = new PIDController(kP, kI, kD);
        
      SparkMaxConfig globalConfig = new SparkMaxConfig();
      SparkMaxConfig rightConfig = new SparkMaxConfig();
      SparkMaxConfig encoderConfig = new SparkMaxConfig();

      
      globalConfig
          .smartCurrentLimit(50)
          .idleMode(IdleMode.kBrake);

      // Apply the global config and invert since it is on the opposite side
      rightConfig
          .apply(globalConfig)
          .inverted(true)
          .follow(leftMotor);

      encoderConfig.encoder
          .positionConversionFactor(ElevatorConstants.CONVERSION_FACTOR)
          .velocityConversionFactor(ElevatorConstants.CONVERSION_FACTOR / 60);
          
      /*
      * Apply the configuration to the SPARKs.
      *
      * kResetSafeParameters is used to get the SPARK MAX to a known state. This
      * is useful in case the SPARK MAX is replaced.
      *
      * kPersistParameters is used to ensure the configuration is not lost when
      * the SPARK MAX loses power. This is useful for power cycles that may occur
      * mid-operation.
      */
      leftMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      encoder.reset();
    }
    
    public void setHeight(int heightIndex) {
      if (heightIndex < 0 || heightIndex >= HEIGHTS.length) return;
      double targetPosition = HEIGHTS[heightIndex];
      double output = pidController.calculate(encoder.getDistance(), targetPosition);
      leftMotor.set(output);
    }

    public boolean isAtHeight(int heightIndex) {
      return Math.abs(encoder.getDistance() - HEIGHTS[heightIndex]) < ERROR_MARGIN;
    }

    public void stop() {
      leftMotor.set(0);
   }
}

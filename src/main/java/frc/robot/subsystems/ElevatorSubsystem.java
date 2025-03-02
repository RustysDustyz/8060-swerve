package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final Encoder encoder;

    public ElevatorSubsystem() {
        
      leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
      rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);
      encoder = new Encoder(ElevatorConstants.encoderChannelA, ElevatorConstants.encoderChannelB);
      SparkMaxConfig globalConfig = new SparkMaxConfig();
      SparkMaxConfig rightConfig = new SparkMaxConfig();
      SparkMaxConfig encoderConfig = new SparkMaxConfig();

      
      /*These don't work
      globalConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast);

      // Apply the global config and invert since it is on the opposite side
      rightConfig
        .idleMode(IdleMode.kCoast)
        .follow(leftMotor)
        .inverted(true);
      */

      encoderConfig.encoder
        .positionConversionFactor((Math.PI * 0.058) / 60.0)
        .velocityConversionFactor(((Math.PI * 0.058) / 60.0) / 60);
          
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

    public void move(double speed){
      leftMotor.set(speed);
      rightMotor.set(-speed);
    }

    public void stop() {
      leftMotor.set(0);
   }
}

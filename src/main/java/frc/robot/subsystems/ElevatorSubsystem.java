package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Motors & Encoder
  private final SparkMax rightMotor = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);
  private final SparkMax leftMotor = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = rightMotor.getEncoder();

  private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  //private final SparkBaseConfig motor1Config;

  // Motion Controllers
  private final ProfiledPIDController m_controller = 
      new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 
                                ElevatorConstants.CONSTRAINTS, 0.02);
  private final ElevatorFeedforward m_feedforward = 
      new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

  public ElevatorSubsystem() {

    motorConfig.encoder
      .positionConversionFactor(ElevatorConstants.CONVERSION_FACTOR)
      .velocityConversionFactor(getElevatorPositionMeters() / 60);
    
    

    // Configure the follower motor
    leftMotor.follow(rightMotor); // Follower motor mimics the lead motor
    leftMotor.setInverted(true);   // Invert if needed (depends on physical setup)

    // Configure motor settings (optional)
    rightMotor.setSmartCurrentLimit(40);
    leftMotor.setSmartCurrentLimit(40);
  }

  public double getElevatorPositionMeters() {
    return m_encoder.getPosition(); // Convert motor rotations to meters
  }

  public double getElevatorVelocityMetersPerSecond() {
    return m_encoder.getVelocity(); // Convert RPM to m/s
  }

  public void setElevatorGoal(double goal) {
    m_controller.setGoal(goal);
  }

  public void updateElevator() {
    double output = m_controller.calculate(getElevatorPositionMeters())
                    + m_feedforward.calculate(m_controller.getSetpoint().velocity);
    
    // Clamp output to safe voltage range
    output = Math.max(Math.min(output, 12), -12);
    rightMotor.setVoltage(output); // Follower automatically follows lead motor

    // Stop motor when within 1 cm of goal
    if (Math.abs(m_controller.getGoal().position - getElevatorPositionMeters()) < 0.01) {
        stopElevator();
    }
  }

  public void stopElevator() {
    rightMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // Display encoder readings on SmartDashboard
    SmartDashboard.putNumber("Elevator Height (m)", getElevatorPositionMeters());
    SmartDashboard.putNumber("Elevator Velocity (m/s)", getElevatorVelocityMetersPerSecond());
  }
}
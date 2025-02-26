package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private static double kDt = 0.02;
  private static double kMaxVelocity = 1.75;
  private static double kMaxAcceleration = 0.75;
  private static double kP = 1.3;
  private static double kI = 0.0;
  private static double kD = 0.7;
  private static double kS = 1.1;
  private static double kG = 1.2;
  private static double kV = 1.3;

  private final Encoder m_encoder = new Encoder(0, 1);
  private final PWMSparkMax m_motor = new PWMSparkMax(1);

  // PID and feedforward controllers
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

  public ElevatorSubsystem() {
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
  }

  public void setElevatorGoal(double goal) {
    m_controller.setGoal(goal);
  }

  public void updateElevator() {
    // Run the controller and apply voltage to the motor
    m_motor.setVoltage(
        m_controller.calculate(m_encoder.getDistance())
            + m_feedforward.calculate(m_controller.getSetpoint().velocity));
  }

  public double getEncoderDistance() {
    return m_encoder.getDistance();
  }
}

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static final class ElevatorConstants {
        // TODO: Remove this once fully implemented & fix errors. If only testing, DO NOT REMOVE, set to true!
        public static final boolean implemented = false; // Set to true since it's now integrated

        /* Motor IDs */
        public static final int leftMotorID =  1;
        public static final int rightMotorID = 2; 

        /* Encoder Channel */

        public static final int encoderChannelA = 0;
        public static final int encoderChannelB = 1;

        /* Gear Ratio & Pulley System */
        public static final double GEAR_RATIO = 60.0;  // 1:60 gearbox
        public static final double PULLEY_DIAMETER = 0.0508;  // 3.81 cm (1.5 inches)
        public static final double PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIAMETER;
        public static final double CONVERSION_FACTOR = PULLEY_CIRCUMFERENCE / GEAR_RATIO; // Meters per motor rotation

        /* Motion Constraints */
        public static final double MAX_VELOCITY = 0.50;  // Meters per second
        public static final double MAX_ACCELERATION = 0.25; // Meters per second squared
        public static final TrapezoidProfile.Constraints CONSTRAINTS =
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        /* PID Constants */
        public static final double kP = 1.3;
        public static final double kI = 0.0;
        public static final double kD = 0.7;

        /* Feedforward Constants */
        public static final double kS = 1.1;
        public static final double kG = 1.2;
        public static final double kV = 1.3;

    }
}

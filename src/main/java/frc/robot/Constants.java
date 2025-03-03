package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double translationDeadband = 0.1;
    public static final double angularDeadband = 0.5;

    public static final class DriverConstants {
        /**
         * SysID button is an interface for performing SysID functions;
         * see {@link RobotContainer.sysidMode}.
         * Warning: SysID routines make the wheels spin VERY fast.
         * Give the robot LOTS of clearance (don't run SysID inside).
         */
        public static final boolean enableSysID = false;
    }

    public static final class SwerveConstants {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(24.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 0.5; //TODO: This must be tuned to specific robot
        public static final double maxAccel = 0.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 1.0; //TODO: This must be tuned to specific robot
        public static final double maxAngularAccel = 0.5;
        /* Global angle offset */
        public static final Rotation2d globalModuleAngleOffset = Rotation2d.fromDegrees(0);

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Options */
        // If wheels should prioritize reversing over turning in optimization.
        public static boolean optimizeWheelReverse = true;

        public static final class CameraConstants {
            public static final boolean enabled = false;
        }

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.0512);
            public static final boolean sineCompensation = false;
            public static final boolean reversed = false;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, sineCompensation, reversed);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.3781);
            public static final boolean sineCompensation = false;
            public static final boolean reversed = true;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, sineCompensation, reversed);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.0627);
            public static final boolean sineCompensation = false;
            public static final boolean reversed = false;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, sineCompensation, reversed);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.2639);
            public static final boolean sineCompensation = true;
            public static final boolean reversed = false;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, sineCompensation, reversed);
        }
    }

    public static final class ElevatorConstants {
        // TODO: Remove this once fully implemented & fix errors. If only testing, DO NOT REMOVE, set to true!
        public static final boolean implemented = false; // Set to true since it's now integrated

        /* Motor IDs */
        public static final int leftMotorID = 1;
        public static final int rightMotorID = 2; 
        public static final int wristMotorID = 3;

        /* Encoder Channel */

        public static final int encoderChannelA = 2;
        public static final int encoderChannelB = 3;

        /* Gear Ratio & Pulley System */
        public static final double GEAR_RATIO = 60.0;  // 1:60 gearbox
        public static final double PULLEY_DIAMETER = 0.0381;  // 3.81 cm (1.5 inches)
        public static final double PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIAMETER;
        public static final double CONVERSION_FACTOR = PULLEY_CIRCUMFERENCE / GEAR_RATIO; // Meters per motor rotation

        /* Motion Constraints */
        public static final double MAX_VELOCITY = 0.1;  // Meters per second
        public static final double MAX_ACCELERATION = 0.05;; // Meters per second squared
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

        /* Elevator Setpoints (in meters) */
        public static final int setpointCommandCount = 5;
        public static final double ELEVATOR_SETPOINT_1 = 0.1;
        public static final double ELEVATOR_SETPOINT_2 = 0.3;
        public static final double ELEVATOR_SETPOINT_3 = 0.6;
        public static final double ELEVATOR_SETPOINT_4 = 0.9;
        public static final double ELEVATOR_SETPOINT_5 = 1.2;

    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 0.2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 0.2;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 0.2;
        // The speed at which the robot drives during autonomous mode in comparison to in
        // teleoperated mode.
        public static final double speedFactor = 0.06;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}

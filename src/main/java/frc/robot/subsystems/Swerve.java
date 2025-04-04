package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.concurrent.Flow.Publisher;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    private boolean leftRight = false;
    private Publisher p_field;
    
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    
    public ADXRS450_Gyro gyro;

    public final StructArrayPublisher<SwerveModuleState> publisher;
    public final Field2d field;
    /*
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
    */

    private SysIdRoutine m_driveSysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts -> {
                for (SwerveModule swerveModule : mSwerveMods)
                swerveModule.setDriveVoltage(volts.in(BaseUnits.VoltageUnit));
            },
            log -> {
                for(SwerveModule mod : mSwerveMods){
                    SignalLogger.writeDouble(String.format("mod%d-voltage",mod.moduleNumber), mod.getDriveMotor().get() * RobotController.getBatteryVoltage(),"V");
                    SignalLogger.writeDouble(String.format("mod%d-distance",mod.moduleNumber), mod.getPosition().distanceMeters,"m");
                    SignalLogger.writeDouble(String.format("mod%d-velocity",mod.moduleNumber), mod.getState().speedMetersPerSecond,"m/s");
                }
            },
            this
        ));

    public Swerve() {
        //SignalLogger.setPath("/home/lvuser/logs/");
        gyro = new ADXRS450_Gyro();
        gyro.reset();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
        publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getCANcoder().getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getCANcoder().getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond, null);
                
                builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getCANcoder().getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getCANcoder().getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> mSwerveMods[3].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
            }
        });

        field = new Field2d();
        SmartDashboard.putData("Field",field);

        SmartDashboard.putNumber("Heading",0);

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> drive(speeds.times(AutoConstants.speedFactor)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController(
                new PIDConstants(
                    Constants.SwerveConstants.driveKP, 
                    Constants.SwerveConstants.driveKI,
                    Constants.SwerveConstants.driveKD
                ),
                new PIDConstants(
                    Constants.SwerveConstants.angleKP, 
                    Constants.SwerveConstants.angleKI, 
                    Constants.SwerveConstants.angleKD
                )
            ),
            Robot.robotConfig, // The robot configuration
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
        );
    }

    public ChassisSpeeds getChassisSpeeds(){
        return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    

    public double rotAimAssist(){
        double kP_rot = 0.0075; // Tune this value based on testing
        //•	  If the robot overshoots, reduce kP.
        //•	  If the robot is too slow, increase kP.
        double tx = LimelightHelpers.getTX("limelight"); // Horizontal error
        double rotationSpeed = tx * kP_rot * Constants.SwerveConstants.maxAngularVelocity;
        
        return rotationSpeed;
    }

    public Translation2d transAimAssist() {
        double kP_trans = 0.2; // Adjust for responsiveness
        double targetOffsetMeters = 8 * 0.0254; // 8 inches in meters
        double limelightOffset = 8 * 0.0254; // Limelight is 8 inches to the left
    
        Pose2d aprilTagPose = LimelightHelpers.getBotPose2d("limelight");
        
        // Compute the true robot center Y position
        double currentY = aprilTagPose.getY() + limelightOffset;
    
        // Determine target Y position (8 inches left or right of the tag)
        double targetY = aprilTagPose.getY() + (leftRight ? targetOffsetMeters : -targetOffsetMeters);
        System.out.println("Strafe Speed: " + targetY);
    
        // Compute error (difference between current and target)
        double error = targetY - currentY;
        //System.out.println("Error: " + error);
    
        // Stop moving when close enough
        if (Math.abs(error) < 0.01) { // Within 1cm of target
            //System.out.println("Close enough, stopping");
            return new Translation2d(0, 0);
        }
    
        // Move towards the target
        double strafeSpeed = error * kP_trans;
        //System.out.println("Strafe Speed: " + strafeSpeed);
    
        return new Translation2d(0, strafeSpeed);
    }

    public void leftRightAim(){
        leftRight = !leftRight;
    }

    public void drive(ChassisSpeeds speeds){
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            // set desired state
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean rotAssist, boolean transAssist) {

        if (rotAssist) {
            rotation = rotAimAssist();

            //System.out.printf("r: %.3f",rotation);
            fieldRelative = false; // Disable field-relative while aiming
        }

        if (transAssist) {
            translation = transAimAssist();

            //System.out.printf("t: %s",translation);
            // we could also do this:
            // translation = translation.plus(trans_aimAssist());
            fieldRelative = false; // Disable field-relative while aiming
        }
        
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getHeading()
                )
                : new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            if(Math.abs(rotation) > 0.001){
                // fix modules
                if(mod.moduleNumber == 2)
                    swerveModuleStates[mod.moduleNumber].angle = swerveModuleStates[mod.moduleNumber].angle.rotateBy(
                        Rotation2d.kCCW_Pi_2);
                if(mod.moduleNumber == 3)
                    swerveModuleStates[mod.moduleNumber].angle = swerveModuleStates[mod.moduleNumber].angle.rotateBy(
                        Rotation2d.kCW_Pi_2);
                
                // un-global offset for z-axis
                swerveModuleStates[mod.moduleNumber].angle = 
                    swerveModuleStates[mod.moduleNumber].angle.rotateBy(Constants.SwerveConstants.globalModuleAngleOffset.times(-1));
            }

            // set desired state
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation().times(-1);
    }

    public SwerveModule[] getModules(){
        return mSwerveMods;
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Command getDriveQuadTest(){
        return m_driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command getDriveDynamTest(){
        return m_driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }    

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    }
}
package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;

    private boolean optReverse = false;
    private boolean highDelta = false;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);
    private final PIDController turningPidController;

    private SwerveModuleState optimize(SwerveModuleState state, Rotation2d currentAngle){
        double delta = state.angle.getDegrees() - (currentAngle.getDegrees() - angleOffset.getDegrees()) - Constants.Swerve.globalModuleAngleOffset.getDegrees();
        //if(optReverse) delta -= 180;
        delta = Math.abs(delta);
        System.out.printf("delta: %.4f\na: %.4f/%.4f\nr: %b\n",delta,state.angle.getDegrees(),currentAngle.getDegrees(),optReverse);
        if(!highDelta && delta > 90.0) {
            highDelta = true;
            optReverse = !optReverse;
        }else if(highDelta && delta < 80.0){
            highDelta = false;
            optReverse = !optReverse;
        }
        if(optReverse){
            state.speedMetersPerSecond *= -1;
            state.angle = state.angle.rotateBy(Rotation2d.kPi);
        }

        /*var delta = state.angle.minus(currentAngle);
        System.out.printf("delta: %.4f\n",Math.abs(delta.getDegrees()))
        if (Math.abs(delta.getDegrees()) > 90.0) {
            optReverse = true;
        }
        if(optReverse){
            state.speedMetersPerSecond *= -1;
            state.angle = state.angle.rotateBy(Rotation2d.kPi);
        }*/

        return state;
    }

    public void resetOptimization(){
        optReverse = false;
    }

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        turningPidController = new PIDController(0.5, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        //if(angleEncoder.getDeviceID() == 4) System.out.printf("pre-op: %s\n",desiredState.toString());
        SwerveModuleState optimizedState = optimize(desiredState,getState().angle);
        //if(angleEncoder.getDeviceID() == 4) System.out.printf("post-op: %s\n",desiredState.toString());
        mAngleMotor.set(turningPidController.calculate(
            mAngleMotor.getPosition().getValueAsDouble(),
            optimizedState.angle.getRotations() + Constants.Swerve.globalModuleAngleOffset.getRotations()
        ));
        setSpeed(optimizedState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        //System.out.println(desiredState);
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }
}
package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
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
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    public int moduleNumber;

    private Rotation2d angleOffset;
    private SwerveModuleState lastDesiredState;
    private long rotationOffset = 0;
    private boolean reversed = false;
    private int reverseRotOffset = 0;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private SwerveModuleConstants moduleConstants;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PIDController turningPidController;

    public void resetOptimization(){
        rotationOffset = 0;
        reverseRotOffset = 0;
        reversed = false;
        lastDesiredState = null;
    }

    private void optimize(SwerveModuleState desiredState){
        if(lastDesiredState == null){
            lastDesiredState = desiredState;
            return;
        }
        double valueDelta = desiredState.angle.getDegrees() - lastDesiredState.angle.getDegrees();
        double lastDesiredAngle = lastDesiredState.angle.getDegrees() + rotationOffset*360;

        if(Math.abs(valueDelta) > 180){
            rotationOffset -= Math.signum(valueDelta);
        }
        double stateDelta = (desiredState.angle.getDegrees() + rotationOffset*360) - lastDesiredAngle;
        //System.out.printf("sd: %.2f\nr: %b\n",stateDelta,reversed);
        if(SwerveConstants.optimizeWheelReverse){
            if(Math.abs(stateDelta) > 90){
                reversed = !reversed;
                reverseRotOffset += (int)Math.signum(stateDelta)/2;
            }
        }
        lastDesiredState = desiredState;
        if(reversed){
            desiredState.speedMetersPerSecond *= -1;
            desiredState.angle = desiredState.angle.rotateBy(Rotation2d.kPi);
        }
    }

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        this.moduleConstants = moduleConstants;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        turningPidController = new PIDController(0.5, 0, 0);
        turningPidController.enableContinuousInput(Math.PI, Math.PI);

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
        optimize(desiredState);
        
        mAngleMotor.set(turningPidController.calculate(
            mAngleMotor.getPosition().getValueAsDouble(),
            desiredState.angle.getRotations()
                + rotationOffset - reverseRotOffset
                + Constants.SwerveConstants.globalModuleAngleOffset.getRotations()
                
        ));

        if(moduleConstants.sineCompensation){
            desiredState.speedMetersPerSecond *= desiredState.angle.minus(getCANcoder()).getSin() * -1;
        }else{
            desiredState.speedMetersPerSecond *= desiredState.angle.minus(getCANcoder()).getCos();
        }
        if(moduleConstants.reversed) desiredState.speedMetersPerSecond *= -1;
        //desiredState.cosineScale(getCANcoder());
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        //System.out.println(desiredState);
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            //mDriveMotor.set(driveDutyCycle.Output);
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            //mDriveMotor.set(driveVelocity.FeedForward);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public TalonFX getDriveMotor(){
        return mDriveMotor;
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public void setDriveVoltage(double voltage) {
        driveDutyCycle.Output = voltage;
        mDriveMotor.setControl(driveDutyCycle);
    }
}
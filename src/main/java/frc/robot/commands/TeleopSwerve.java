package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private final SlewRateLimiter translationLimiter, strafeLimiter, rotationLimiter;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        // Initialize slew rate limiters for x, y, and turning speeds
        this.translationLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        this.strafeLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        this.rotationLimiter = new SlewRateLimiter(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        translationVal = translationLimiter.calculate(translationVal) * Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
        strafeVal = strafeLimiter.calculate(strafeVal) * Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
        rotationVal = rotationLimiter.calculate(rotationVal) * Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared;

        rotationVal = 0;

        //System.out.printf("t:%.3f, s:%.3f, r:%.3f\n",translationVal,strafeVal,rotationVal);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(-strafeVal, translationVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
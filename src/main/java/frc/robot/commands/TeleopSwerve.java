package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
    private final Supplier<Boolean> aimAssist;
    private final SlewRateLimiter translationLimiter, strafeLimiter, rotationLimiter;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, Supplier<Boolean> aimAssist) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.aimAssist = aimAssist;

        // Initialize slew rate limiters for x, y, and turning speeds
        this.translationLimiter = new SlewRateLimiter(Constants.SwerveConstants.maxAccel);
        this.strafeLimiter = new SlewRateLimiter(Constants.SwerveConstants.maxAccel);
        this.rotationLimiter = new SlewRateLimiter(Constants.SwerveConstants.maxAngularAccel);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.translationDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.translationDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.angularDeadband);

        translationVal = translationLimiter.calculate(translationVal) * Constants.SwerveConstants.maxAccel;
        strafeVal = strafeLimiter.calculate(strafeVal) * Constants.SwerveConstants.maxAccel;
        rotationVal = rotationLimiter.calculate(rotationVal) * Constants.SwerveConstants.maxAngularAccel;

        //System.out.printf("t:%.3f, s:%.3f, r:%.3f\n",translationVal,strafeVal,rotationVal);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(-strafeVal, translationVal).times(Constants.SwerveConstants.maxSpeed), 
            rotationVal * Constants.SwerveConstants.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true,
            aimAssist.get()
        );
    }
}
package frc.robot.auto;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve) { //(HARD CODED TRAJECTORY)
    // public exampleAuto(Swerve s_Swerve, Pose2d start, Pose2d end, List<Translation2d> waypoints) {
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.SwerveConstants.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)), //(HARD CODED TRAJECTORY)
                //start,
                // Pass through the interior waypoints, making an 's' curve path
                List.of(
                    //new Translation2d(0.2, 0.2), 
                    new Translation2d(1.0, 0)
                ), //(HARD CODED TRAJECTORY)
                //waypoints,
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1.0,0, Rotation2d.fromDegrees(0)), //(HARD CODED TRAJECTORY)
                //end,
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

// Paco was Here
        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}
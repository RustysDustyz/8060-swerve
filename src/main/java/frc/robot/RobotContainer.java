package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /* ShuffleboardTab Inputs */
    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Trajectory Config");
    private final GenericEntry startX = autoTab.add("Start X", 0.0).getEntry();
    private final GenericEntry startY = autoTab.add("Start Y", 0.0).getEntry();
    private final GenericEntry startTheta = autoTab.add("Start Theta", 0.0).getEntry();
    private final GenericEntry endX = autoTab.add("End X", 3.0).getEntry();
    private final GenericEntry endY = autoTab.add("End Y", 0.0).getEntry();
    private final GenericEntry endTheta = autoTab.add("End Theta", 0.0).getEntry();

     final List<GenericEntry> waypointX = new ArrayList<>();
    private final List<GenericEntry> waypointY = new ArrayList<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Add Waypoint Entries to Shuffleboard
        for (int i = 0; i<5; i++) {
            waypointX.add(autoTab.add("Waypoint " + (i + 1) + " X", 0.0).getEntry());
            waypointY.add(autoTab.add("Waypoint " + (i + 1) + " Y", 0.0).getEntry());
        }

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous

        // Read Inputs from Shuffleboard
        Pose2d startPose = new Pose2d(
            startX.getDouble(0.0),
            startY.getDouble(0.0),
            Rotation2d.fromDegrees(startTheta.getDouble(0.0))
        );
        Pose2d endPose = new Pose2d(
            endX.getDouble(3.0),
            endY.getDouble(0.0),
            Rotation2d.fromDegrees(endTheta.getDouble(0.0))
        );

        List<Translation2d> waypoints = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            double x = waypointX.get(i).getDouble(0.0);
            double y = waypointY.get(i).getDouble(0.0);

            // Add non-zero waypoints to the trajectory
            if (x != 0.0 || y != 0.0) {
                waypoints.add(new Translation2d(x, y));
            }
        }

        return new exampleAuto(s_Swerve, startPose, endPose, waypoints);
        // return new exampleAuto(s_Swerve); (HARD CODED TRAJECTORY)
    }
}

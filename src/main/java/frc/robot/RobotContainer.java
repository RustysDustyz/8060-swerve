package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kLeftTrigger.value;

    /* Driver Buttons */
    // Buttons labelled by numbers on the LogiTech Extreme
    private final JoystickButton translationMode = new JoystickButton(driver, 1);
    private final JoystickButton sysidMode = new JoystickButton(driver, 2);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4);
    private final JoystickButton robotCentric = new JoystickButton(driver, 5);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /* Shuffleboard */
    //private final ShuffleboardTab tab = Shuffleboard.getTab("Auto");
    //private final GenericEntry placeholder = tab.add("Placeholder",1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // SmartDashboard Inputs
        SmartDashboard.putNumber("startX", 0.5);
        SmartDashboard.putNumber("startY", 0.5);
        SmartDashboard.putNumber("startTheta", 0.0);
        SmartDashboard.putNumber("endX", 1.0);
        SmartDashboard.putNumber("endY", 1.0);
        SmartDashboard.putNumber("endTheta", 0.0);
        
        // Add Waypoint Entries to SmartDashboard
        for (int i = 0; i<5; i++) {
            SmartDashboard.putNumber("waypointX-%d".formatted(i), 0.0);
            SmartDashboard.putNumber("waypointY-%d".formatted(i), 0.0);
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
        translationMode.toggleOnTrue(new InstantCommand(() -> s_Swerve.toggleTransMode()));
        sysidMode
            .and(new JoystickButton(driver, 7))
            .toggleOnTrue(new InstantCommand(s_Swerve::getDriveDynamTest));
            sysidMode
            .and(new JoystickButton(driver, 8))
            .toggleOnTrue(new InstantCommand(s_Swerve::getDriveQuadTest));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        /*
        // Read Inputs from Shuffleboard
        SmartDashboard.updateValues();

        Pose2d startPose = new Pose2d(
            SmartDashboard.getNumber("startX", 0.5),
            SmartDashboard.getNumber("startY", 0.5),
            Rotation2d.fromDegrees(SmartDashboard.getNumber("startTheta", 0.0))
        );
        Pose2d endPose = new Pose2d(
            SmartDashboard.getNumber("endX", 1.0),
            SmartDashboard.getNumber("endY", 1.0),
            Rotation2d.fromDegrees(SmartDashboard.getNumber("endTheta", 0.0))
        );
        System.out.print("start: ");
        System.out.println(startPose);
        System.out.print("end: ");
        System.out.println(endPose);

        // Program is prone to crash if start/end are too close in the path.
        // If that is true, the program will return a null command instead.
        if(startPose.getTranslation().getDistance(endPose.getTranslation()) < 0.01) return null;

        List<Translation2d> waypoints = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            double x = SmartDashboard.getNumber("waypointX-%d".formatted(i), 0.0);
            double y = SmartDashboard.getNumber("waypointY-%d".formatted(i), 0.0);

            System.out.printf("w[%d]: ",i);
            System.out.printf("Translation2d(X: %.2f, Y: %.2f)\n",x,y);

            // Add non-zero waypoints to the trajectory
            if (x != 0.0 || y != 0.0) {
                waypoints.add(new Translation2d(x, y));
            }
        }
        
        return new exampleAuto(s_Swerve, startPose, endPose, waypoints);
        */
        return new exampleAuto(s_Swerve); //(HARD CODED TRAJECTORY)
    }
}

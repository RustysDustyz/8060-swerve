package frc.robot;

import com.ctre.phoenix6.controls.MusicTone;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ElevatorConstants;
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

    // TODO: Implement an "aim assist" system for AprilTags using LimeLight.
    private final JoystickButton aimAssist = new JoystickButton(driver, 2);

    private final JoystickButton sysidInterface = new JoystickButton(driver, 3);
    private final JoystickButton featureTestInterface = new JoystickButton(driver, 4);
    
    private final JoystickButton robotCentric = new JoystickButton(driver, 5);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 6);

    private final Trigger notInterface = sysidInterface.or(featureTestInterface).negate();

    /* Subsystems */
    private final LimelightConfig s_LimelightConfig = new LimelightConfig();
    private final Swerve s_Swerve = new Swerve();
    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();

    /* Elevator Control Buttons */
    private final JoystickButton elevatorButton1 = new JoystickButton(driver, 7);  // Assign buttons
    private final JoystickButton elevatorButton2 = new JoystickButton(driver, 8);
    private final JoystickButton elevatorButton3 = new JoystickButton(driver, 9);
    private final JoystickButton elevatorButton4 = new JoystickButton(driver, 10);
    private final JoystickButton elevatorButton5 = new JoystickButton(driver, 11);


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
        /*for (int i = 0; i<5; i++) {
            SmartDashboard.putNumber("waypointX-%d".formatted(i), 0.0);
            SmartDashboard.putNumber("waypointY-%d".formatted(i), 0.0);
        }*/

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> aimAssist.getAsBoolean()
            )
        );
        s_Elevator.setDefaultCommand(
            new RunCommand(
                () -> s_Elevator.updateElevator(), 
                s_Elevator
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
        
        /* Elevator Setpoints */
        elevatorButton1.onTrue(new ElevatorSetpointCommand(s_Elevator, ElevatorConstants.ELEVATOR_SETPOINT_1));
        elevatorButton2.onTrue(new ElevatorSetpointCommand(s_Elevator, ElevatorConstants.ELEVATOR_SETPOINT_2));
        elevatorButton3.onTrue(new ElevatorSetpointCommand(s_Elevator, ElevatorConstants.ELEVATOR_SETPOINT_3));
        elevatorButton4.onTrue(new ElevatorSetpointCommand(s_Elevator, ElevatorConstants.ELEVATOR_SETPOINT_4));
        elevatorButton5.onTrue(new ElevatorSetpointCommand(s_Elevator, ElevatorConstants.ELEVATOR_SETPOINT_5));
        
        /* Driver Buttons */

        // Zero Gyro : Press or hold Btn 6 to reset heading
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // Translation Mode : Press Btn 1 to toggle
        translationMode.onTrue(new InstantCommand(() -> s_Swerve.toggleTransMode()));

        if(DriverConstants.enableSysID){
            // Sys ID Dynam Test : Press Btn 3 + 7 to start
            // Warning: Very fast!
            sysidInterface
                .and(new JoystickButton(driver, 7))
                .onTrue(s_Swerve.getDriveDynamTest());
                
            // SYS ID Quad Test : Press Btn 3 + 8 to start
            // Warnin g: Very fast!
            sysidInterface
                .and(new JoystickButton(driver, 8))
                .onTrue(s_Swerve.getDriveQuadTest());
        }else{
            sysidInterface.onTrue(new InstantCommand(() -> System.out.println(
                "SysID interface is disabled. Enable in Constants.DriverConstants.enableSysID"
            )));
        }

        if(DriverConstants.enableFeatureTest){
            // Feature Test - Move test : Press Btn 4 + 7 to start
            featureTestInterface
                .and(new JoystickButton(driver, 7))
                .onTrue(
                    new RunCommand(
                        () -> {for(SwerveModule m : s_Swerve.getModules()){
                            m.getDriveMotor().set(0.25);
                        }}
                    ).until(new JoystickButton(driver, 7).negate())
                );

            // Feature Test - Reset motor optimization : Press Btn 4 + 8 to start
            featureTestInterface
                .and(new JoystickButton(driver, 8))
                .onTrue(
                    new InstantCommand(
                        () -> {for(SwerveModule m : s_Swerve.getModules()){
                            m.getDriveMotor().setControl(new MusicTone(880));
                            m.resetOptimization();
                        }}
                    )
                );
        }else{
            featureTestInterface.onTrue(new InstantCommand(() -> System.out.println(
                "Feature Test interface is disabled. Enable in Constants.DriverConstants.enableFeatureTest"
            )));
        }

        // Non-interface buttons
        new JoystickButton(driver, 7)
            .and(notInterface)
            .onTrue(new InstantCommand(() -> System.out.println("Base btn 7")));
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

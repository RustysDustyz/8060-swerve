package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.ElevatorConstants;
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
    private final int translationAxis = XboxController.Axis.kLeftY.value; // 1
    private final int strafeAxis = XboxController.Axis.kLeftX.value; // 0
    private final int rotationAxis = XboxController.Axis.kLeftTrigger.value; // 2

    /* Driver Buttons */
    // Buttons labelled by numbers on the LogiTech Extreme
    private final JoystickButton translationMode = new JoystickButton(driver, 3);

    // TODO: Implement an "aim assist" system for AprilTags using LimeLight.
    private final JoystickButton rotAssist = new JoystickButton(driver, 1);
    private final JoystickButton transAssist = new JoystickButton(driver, 4);
    
    private final JoystickButton robotCentric = new JoystickButton(driver, 5);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 6);


    /* Subsystems */
    private final LimelightConfig s_LimelightConfig = new LimelightConfig();
    private final Swerve s_Swerve = new Swerve();
    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    private final WristSubsystem s_Wrist = new WristSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // SmartDashboard Inputs
        SmartDashboard.putString("autoCommand", "test_elev");

        /*
        SmartDashboard.putNumber("startX", 0.5);
        SmartDashboard.putNumber("startY", 0.5);
        SmartDashboard.putNumber("startTheta", 0.0);
        SmartDashboard.putNumber("endX", 1.0);
        SmartDashboard.putNumber("endY", 1.0);
        SmartDashboard.putNumber("endTheta", 0.0);
        */
        
        // Add Waypoint Entries to SmartDashboard
        /*for (int i = 0; i<5; i++) {
            SmartDashboard.putNumber("waypointX-%d".formatted(i), 0.0);
            SmartDashboard.putNumber("waypointY-%d".formatted(i), 0.0);
        }*/

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawButton(2) ? 0 : driver.getRawAxis(translationAxis), 
                () -> driver.getRawButton(2) ? 0 : driver.getRawAxis(strafeAxis), 
                () -> driver.getRawButton(2) ? 0 : driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> rotAssist.getAsBoolean(),
                () -> transAssist.getAsBoolean()
            )
        );

        s_Wrist.setDefaultCommand(
            new WristCommand(
                s_Wrist,
                () -> {
                    if(driver.getRawButton(2)) return 0;
                    switch(driver.getPOV()){
                    case 90:
                        return 1;
                    case 270:
                        return -1;
                    default:
                        return 0;
                    }
                },
                () -> !driver.getRawButton(2) ? 0 : driver.getRawAxis(translationAxis)
            )
        );

        

        s_Elevator.setDefaultCommand(
            new IOCommand(
                s_Elevator,
                () -> {switch(driver.getPOV()){
                    case 0:
                        return 1;
                    case 180:
                        return -1;
                    default:
                        return 0;
                }}
            )
        );

        /* Named Commands (Auto) */
        NamedCommands.registerCommand("rotAim", new RunCommand(() -> s_Swerve.drive(Translation2d.kZero, 0, true, true, true, false)).withTimeout(1));
        NamedCommands.registerCommand("transAim", new RunCommand(() -> s_Swerve.drive(Translation2d.kZero, 0, true, true, false, true)).withTimeout(1));
        NamedCommands.registerCommand("dualAim", new RunCommand(() -> s_Swerve.drive(Translation2d.kZero, 0, true, true, true, true)).withTimeout(1));

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
        for(int i=0;i<ElevatorConstants.setpointCommandCount;i++){
            // Creates the command.
            ElevatorSetpointCommand c = new ElevatorSetpointCommand(s_Elevator, s_Wrist, i, i);

            // Registers named commands "elevator<i>"
            NamedCommands.registerCommand(String.format("elevator%d",i), c);

            // Uses every button for <count> iterations, starting from 7.
            new JoystickButton(driver, 7+i).onTrue(c);
        }

        /* Driver Buttons */

        // Zero Gyro : Press or hold Btn 6 to reset heading
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // Translation Mode : Press Btn 1 to toggle
        translationMode.onTrue(new InstantCommand(() -> s_Swerve.toggleTransMode()));
    }

    private void applyAutoLogic(PathPlannerAuto auto, String commandName){
        switch (commandName) {
            case "reefElev1":
                auto.andThen(new RunCommand(() -> {
                    new PathPlannerAuto(driver.getRawButton(12) ? "reefLeft" : "reefRight").execute();
                })).withTimeout(1.5)
                .andThen(new PathPlannerAuto("reefElev2"));
                break;
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        SmartDashboard.updateValues();
        String commandName = SmartDashboard.getString("autoCommand", "t_bezier");
        PathPlannerAuto auto = new PathPlannerAuto(commandName);
        applyAutoLogic(auto, commandName);
        return auto;
    }
}

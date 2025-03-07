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
import frc.robot.Constants.DriverBinds;
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

    /* Driver Buttons */
    // Buttons labelled by numbers on the LogiTech Extreme
    private final JoystickButton translationMode = new JoystickButton(driver, DriverBinds.translationToggleButton);

    private final JoystickButton rotAssist = new JoystickButton(driver, DriverBinds.rotationAimAssistButton);
    private final JoystickButton transAssist = new JoystickButton(driver, DriverBinds.translationAimAssistButton);
    
    private final JoystickButton robotCentric = new JoystickButton(driver, DriverBinds.robotCentricButton);
    private final JoystickButton zeroGyro = new JoystickButton(driver, DriverBinds.zeroGyroButton);
    private final JoystickButton intakeControl = new JoystickButton(driver, DriverBinds.intakeControlButton);

    /* Subsystems */
    @SuppressWarnings("unused")
    private final LimelightConfig s_LimelightConfig = new LimelightConfig();
    private final Swerve s_Swerve = new Swerve();
    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    private final WristSubsystem s_Wrist = new WristSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // SmartDashboard Inputs
        SmartDashboard.putString("autoCommand", "b1_toReef");

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
                () -> intakeControl.getAsBoolean() ? 0 : driver.getRawAxis(DriverBinds.translationAxis), 
                () -> intakeControl.getAsBoolean() ? 0 : driver.getRawAxis(DriverBinds.strafeAxis), 
                () -> intakeControl.getAsBoolean() ? 0 : driver.getRawAxis(DriverBinds.rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> rotAssist.getAsBoolean(),
                () -> transAssist.getAsBoolean()
            )
        );

        s_Wrist.setDefaultCommand(
            new WristCommand(
                s_Wrist,
                () -> {
                    if(intakeControl.getAsBoolean()) return 0;
                    switch(driver.getPOV()){
                    case 90:
                        return 1;
                    case 270:
                        return -1;
                    default:
                        return 0;
                    }
                },
                () -> !intakeControl.getAsBoolean() ? 0 : driver.getRawAxis(DriverBinds.translationAxis)
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
        NamedCommands.registerCommand("intakeIn", new RunCommand(() -> s_Wrist.setIntake(-1)).withTimeout(1));
        NamedCommands.registerCommand("intakeOut", new RunCommand(() -> s_Wrist.setIntake(1)).withTimeout(1));

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
            ElevatorSetpointCommand c = new ElevatorSetpointCommand(s_Elevator, i);

            // Registers named commands "elevator<i>"
            NamedCommands.registerCommand(String.format("elevator%d",i), c);

            // Registers buttons.
            int driverSetpointButtonID = i >= DriverBinds.elevatorSetpointButtons.length
                ? -1
                : DriverBinds.elevatorSetpointButtons[i]
            ;
            if(driverSetpointButtonID != -1) new JoystickButton(driver, driverSetpointButtonID).onTrue(c);
        }

        /* Driver Buttons */

        // Zero Gyro : Press or hold Btn 6 to reset heading
        zeroGyro.and(intakeControl.negate()).onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // Zero Elevator : Press or hold Btn 6 + 2 to reset heading
        zeroGyro.and(intakeControl).onTrue(new InstantCommand(() -> {
            s_Elevator.resetDistance();
            s_Wrist.resetDistance();
        }));

        // Translation Mode : Press Btn 1 to toggle
        translationMode.onTrue(new InstantCommand(() -> s_Swerve.toggleTransMode()));
    }

    private void applyAutoLogic(PathPlannerAuto auto, String commandName){
        /*
        switch (commandName) {
            case "reefElev1":
                auto.andThen(new RunCommand(() -> {
                    new PathPlannerAuto(driver.getRawButton(12) ? "reefLeft" : "reefRight").execute();
                })).withTimeout(1.5)
                .andThen(new PathPlannerAuto("reefElev2"));
                break;
        }
        */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        SmartDashboard.updateValues();
        String commandName = SmartDashboard.getString("autoCommand", "t_bezier");
        System.out.println(commandName);
        PathPlannerAuto auto = new PathPlannerAuto(commandName);
        applyAutoLogic(auto, commandName);
        return auto;
    }
}

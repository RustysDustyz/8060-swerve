package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
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
    private final int wristIO = XboxController.Axis.kRightY.value;

    /* Driver Buttons */
    // Buttons labelled by numbers on the LogiTech Extreme
    private final JoystickButton translationMode = new JoystickButton(driver, 1);

    // TODO: Implement an "aim assist" system for AprilTags using LimeLight.
    private final JoystickButton rotAssist = new JoystickButton(driver, 3);
    private final JoystickButton transAssist = new JoystickButton(driver, 4);

    private final JoystickButton sysidInterface = new JoystickButton(driver, 2);
    
    private final JoystickButton robotCentric = new JoystickButton(driver, 5);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 6);

    private final Trigger notSysID = sysidInterface.negate();


    /* Subsystems */
    private final LimelightConfig s_LimelightConfig = new LimelightConfig();
    private final Swerve s_Swerve = new Swerve();
    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    private final WristSubsystem s_Wrist = new WristSubsystem();

    /* Elevator Control Buttons */
    private final JoystickButton elevatorButton1 = new JoystickButton(driver, 7);  // Assign buttons
    private final JoystickButton elevatorButton2 = new JoystickButton(driver, 8);
    private final JoystickButton elevatorButton3 = new JoystickButton(driver, 9);
    private final JoystickButton elevatorButton4 = new JoystickButton(driver, 10);
    private final JoystickButton elevatorButton5 = new JoystickButton(driver, 11);

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser(); // Build the auto chooser


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        SmartDashboard.putData("Auto Mode", autoChooser);

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
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> rotAssist.getAsBoolean(),
                () -> transAssist.getAsBoolean()
            )
        );

        s_Wrist.setDefaultCommand(
            new IOCommand(
                s_Wrist,
                () -> -driver.getRawAxis(wristIO)
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
        elevatorButton1.and(notSysID).onTrue(new ElevatorSetpointCommand(s_Elevator, s_Wrist, 0, 0));
        elevatorButton2.and(notSysID).onTrue(new ElevatorSetpointCommand(s_Elevator, s_Wrist, 1, 1));
        elevatorButton3.and(notSysID).onTrue(new ElevatorSetpointCommand(s_Elevator, s_Wrist, 2, 2));
        elevatorButton4.and(notSysID).onTrue(new ElevatorSetpointCommand(s_Elevator, s_Wrist, 3, 3));
        elevatorButton5.and(notSysID).onTrue(new ElevatorSetpointCommand(s_Elevator, s_Wrist, 4, 4));

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
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //SmartDashboard.updateValues();

        // return autoChooser.getSelected();
        
        try{
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve(Path.of("pathplanner/paths/p_test01.path"))
            );
            
            AutoBuilder.configure(
                s_Swerve::getPose,
                s_Swerve::setPose,
                s_Swerve::getChassisSpeeds,
                (speeds, feedforwards) -> s_Swerve.drive(speeds.times(0.5)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController(
                    new PIDConstants(
                        Constants.SwerveConstants.driveKP, 
                        Constants.SwerveConstants.driveKI,
                        Constants.SwerveConstants.driveKD
                    ),
                    new PIDConstants(
                        Constants.SwerveConstants.angleKP, 
                        Constants.SwerveConstants.angleKI, 
                        Constants.SwerveConstants.angleKD
                    )
                ),
                Robot.robotConfig, // The robot configuration
                () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                s_Swerve
            );

            ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            SwerveControllerCommand scc = new SwerveControllerCommand(
                trajectory,
                s_Swerve::getPose,
                Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve
            );

            return scc;
            /*if(resetOdomtry){
                return new SequentialCommandGroup(
                    new InstantCommand(() -> s_Swerve.setPose(trajectory.getInitialPose()))
                );
            } else {
                return scc;
            }*/
        }catch(IOException e){
            DriverStation.reportError("Could not load trajectory.", e.getStackTrace());
            return new InstantCommand();
        }
        /*
        PathPlannerAuto auto = new PathPlannerAuto("a_test01");
        return auto;
        */
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightConfig extends SubsystemBase{

    public LimelightConfig () {
        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace("", 
        0.5,    // Forward offset (meters)
        0.0,    // Side offset (meters)
        0.5,    // Height offset (meters)
        0.0,    // Roll (degrees)
        30.0,   // Pitch (degrees)
        0.0     // Yaw (degrees)
        );

        // Set AprilTag offset tracking point (meters)
        LimelightHelpers.setFiducial3DOffset("", 
        0.0,    // Forward offset
        0.0,    // Side offset  
        0.5     // Height offset
        );

        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 2, 3, 4}); // Only track these tag IDs
    }
    
}

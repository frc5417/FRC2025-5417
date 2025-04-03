package frc.robot.subsystems.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PathPlanner {
    //
    // Subsystems
    //
    private final DriveBase m_driveBase;
    private RobotConfig m_robotConfig;
    private boolean isLoaded;

    public PathPlanner(DriveBase driveBase) {
        m_driveBase = driveBase;
        
        try {
            m_robotConfig = RobotConfig.fromGUISettings();

            /* Initialization */
            AutoBuilder.configure(
                m_driveBase.getOdometry()::getPose,
                m_driveBase.getOdometry()::setToPose,
                m_driveBase::getRobotRelativeSpeed,
                (speeds, feedforwards) -> m_driveBase.setRobotRelativeSpeed(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), 
                    new PIDConstants(5.0, 0.0, 0.0)),
                m_robotConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                m_driveBase
            );

            /* Telemetry */
            isLoaded = true;
        } catch (Exception e) {
            e.printStackTrace();
            m_robotConfig = null;
            
            /* Telemetery */
            isLoaded = false;
        }
        SmartDashboard.putBoolean("PathPlanner Loaded", isLoaded);
    }
}

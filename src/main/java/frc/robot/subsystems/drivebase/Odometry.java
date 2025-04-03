package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants.StartPose;

public class Odometry extends SubsystemBase {
    /* Odometry */
    private Field2d field;
    private SendableChooser<Constants.AutonConstants.StartPose> poseSelect; 

    public Odometry() {
        field = new Field2d();

        // /* Starting position select */
        poseSelect = new SendableChooser<Constants.AutonConstants.StartPose>();
        poseSelect.addOption("Blue-Side Barge", StartPose.BLUE_SIDE_BARGE);
        poseSelect.addOption("Red-Side Barge", StartPose.RED_SIDE_BARGE);
        poseSelect.setDefaultOption("Center", StartPose.CENTER);

        // SmartDashboard.putData("Select Position", poseSelect);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Field", field);
    }

    //
    // Position Methods
    //
    public Pose2d getPose() {
        return field.getRobotPose();
    }

    public void setPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    //
    // Telemetry
    //
    /**
     * Uses a sendable to 
     */
    public void setStartPose() {
        /* Initializing variables */
        boolean isRed = getAlliance();
        Pose2d output = new Pose2d(0,0, new Rotation2d(Math.PI));

        /* Selecting the pose */
        switch (poseSelect.getSelected()) {
            case BLUE_SIDE_BARGE:
                output = isRed ? Constants.AutonConstants.kBlueSideBarge_Red : Constants.AutonConstants.kBlueSideBarge_Blue;
                break;
            case RED_SIDE_BARGE:
                output = isRed ? Constants.AutonConstants.kRedSideBarge_Red : Constants.AutonConstants.kRedSideBarge_Blue;
                break;
            case CENTER:
                output = isRed ? Constants.AutonConstants.kCenter_Red: Constants.AutonConstants.kCenter_Blue;
                break;
        }
        field.setRobotPose(output);
    }

    /**
     * 
     * @return {@code true} if red, {@code false} if blue
     */
    public boolean getAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
    
}

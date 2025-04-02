package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
    /* Odometry */
    private Field2d field;

    public Odometry() {
        field = new Field2d();
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

    public void setToPose(Pose2d pose) {
        field.setRobotPose(pose);
    }
    
}

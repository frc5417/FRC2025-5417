package frc.robot.subsystems.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {
    //
    // Modules
    //
    public Module FrontLeft;
    public Module FrontRight;
    public Module BackLeft;
    public Module BackRight;

    private Translation2d m_frontLeftLocation = new Translation2d(Constants.RobotConstants.moduleToOrigin, Constants.RobotConstants.moduleToOrigin);
    private Translation2d m_frontRightLocation = new Translation2d(Constants.RobotConstants.moduleToOrigin, -Constants.RobotConstants.moduleToOrigin);
    private Translation2d m_backLeftLocation = new Translation2d(-Constants.RobotConstants.moduleToOrigin, Constants.RobotConstants.moduleToOrigin);
    private Translation2d m_backRightLocation = new Translation2d(-Constants.RobotConstants.moduleToOrigin, -Constants.RobotConstants.moduleToOrigin);
    
    //
    // Kinematics
    //
    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    private ChassisSpeeds robotSpeeds = new ChassisSpeeds();
    private Gyro m_gyro;

    // Odometry
    public Field2d field = new Field2d();
    private SwerveDrivePoseEstimator m_kalman;
    private RobotConfig robotConfig;
    
    public DriveBase(Gyro gyro) {
        m_gyro = gyro;
        m_gyro.resetYaw();

        FrontLeft = new Module(0);
        FrontRight = new Module(1);
        BackLeft = new Module(2);
        BackRight = new Module(3);

        m_kalman = new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {
            FrontLeft.getPosition(), //fl
            FrontRight.getPosition(), //fr
            BackLeft.getPosition(), //bl
            BackRight.getPosition() //br
        }, new Pose2d(0, 0, new Rotation2d()));

        //
        // Auton
        //
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotSpeeds,
            (speeds, feedforwards) -> setRobotRelativeSpeeds(speeds),
            new PPLTVController(0.02),
            robotConfig,
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            },
            this
        );
    }

    public Pose2d getPose() {
        return m_kalman.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        m_kalman.resetPosition(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                FrontLeft.getPosition(), 
                FrontRight.getPosition(),
                BackLeft.getPosition(), 
                BackRight.getPosition()
            },
            newPose);
    }

    public void setFieldRelativeSpeeds(ChassisSpeeds speeds) {
        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_gyro.getRotation2d()));

        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];

        FrontLeft.setDesiredState(frontLeft, Constants.DriveBaseConstants.isOpenLoop);
        FrontRight.setDesiredState(frontRight, Constants.DriveBaseConstants.isOpenLoop);
        BackLeft.setDesiredState(backLeft, Constants.DriveBaseConstants.isOpenLoop);
        BackRight.setDesiredState(backRight, Constants.DriveBaseConstants.isOpenLoop);
    }

    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        // Convert to module states
        robotSpeeds = speeds;
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];

        FrontLeft.setDesiredState(frontLeft, Constants.DriveBaseConstants.isOpenLoop);
        FrontRight.setDesiredState(frontRight, Constants.DriveBaseConstants.isOpenLoop);
        BackLeft.setDesiredState(backLeft, Constants.DriveBaseConstants.isOpenLoop);
        BackRight.setDesiredState(backRight, Constants.DriveBaseConstants.isOpenLoop);
    }

    @Override
    public void periodic() {
        // Get the rotation of the robot from the gyro.
        m_kalman.update(m_gyro.getRotation2d(), 
        new SwerveModulePosition[] {
            FrontLeft.getPosition(), 
            FrontRight.getPosition(),
            BackLeft.getPosition(), 
            BackRight.getPosition()
        });
        
        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);
    }

    //
    // Get and Set Methods
    //
    public Gyro getGyro() {
        return m_gyro;
    }

    public ChassisSpeeds getRobotSpeeds() {
        return robotSpeeds;
    }
}

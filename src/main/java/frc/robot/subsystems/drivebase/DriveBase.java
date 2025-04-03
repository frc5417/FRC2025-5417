package frc.robot.subsystems.drivebase;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {
    //
    // Drive Variables
    //
    private static Module.ModuleState targetModuleStates[];
    private final Kinematics m_kinematics;
    private final Gyro m_gyro;
    private final Odometry m_odom;
    private final PathPlanner m_pathPlanner;
    public static Module[] moduleGroup;

    //
    // Odometry Variables
    //
    public static double[] odomDeltas = {0, 0, 0, 0};
    public static double[] odomPrevDeltas = {0, 0, 0, 0};
    public static double[] odomAngles = {0, 0, 0, 0};
    public static double[] encoderOffset = {0, 0, 0, 0};
    public static double[] encoderDriveOffset = {0, 0, 0, 0};

    Field2d field = new Field2d();

    double mod1Prev = 0;
    double mod1Curr = 0;
    int counter = 0;

    double tic, toc = 0;

    // might need to change this
    Translation2d m_frontLeftLocation = new Translation2d(-0.23495, 0.23495);
    Translation2d m_frontRightLocation = new Translation2d(0.23495, 0.23495);
    Translation2d m_backLeftLocation = new Translation2d(-0.23495, -0.23495);
    Translation2d m_backRightLocation = new Translation2d(0.23495, -0.23495);

    SwerveDriveKinematics m_skdKine = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    SwerveDriveOdometry m_sdkOdom;    

    ChassisSpeeds autoSetSpeed = new ChassisSpeeds();

    private double[] chassisSpeed_pub = {0.0, 0.0, 0.0};

    public Supplier<double[]> chassisSpeed_supp = ()->chassisSpeed_pub;

    private String alliancecolor_pub = "IDK";

    public Supplier<String> alliancecolor_supp = ()->alliancecolor_pub;


    public DriveBase(Kinematics kinematics, Gyro gyro) {
        /* Movement */
        m_kinematics = kinematics;
        m_gyro = gyro;
        m_odom = new Odometry();
        moduleGroup = new Module[4];
        
        for (int i = 0; i < 4; i++) {
            moduleGroup[i] = new Module(i, Constants.ModuleConstants.invertedDrive[i]);
            encoderOffset[i] = moduleGroup[i].getRadians();
            encoderDriveOffset[i] = moduleGroup[i].integratedDriveEncoder.getPosition();
        }

        targetModuleStates = new Module.ModuleState[4];

        for (int i = 0; i < 4; i++)
            targetModuleStates[i] = new Module.ModuleState(0, Constants.ModuleConstants.angleOffset[i] * (Math.PI/180));

        /* Odometry */
        // most likely needs to be fixed
        m_sdkOdom = new SwerveDriveOdometry(
            m_skdKine, m_gyro.getRotation2d(), new SwerveModulePosition[] {
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0])),
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1]))
            }); 
            
        /* Auton (PathPlanner) */
        m_pathPlanner = new PathPlanner(this);
        m_pathPlanner.initialize();

    } 

    public void resetOdometry(Pose2d pose) {
        Pose2d inverted = new Pose2d(pose.getY(), pose.getX() * -1.0, pose.getRotation());
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = (((moduleGroup[i].integratedDriveEncoder.getPosition() - encoderDriveOffset[i])/6.12) * (0.102*Math.PI));// - odomPrevDeltas[i];
            odomAngles[i] = smallestAngle(moduleGroup[i].getRadians()); //smallestAngle(moduleGroup[i].getAngleInRadians()*(180.0/Math.PI)) * (Math.PI/180.0);
        }
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(odomDeltas[2], new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(odomDeltas[0], new Rotation2d(odomAngles[0])),
                new SwerveModulePosition(odomDeltas[3], new Rotation2d(odomAngles[3])),
                new SwerveModulePosition(odomDeltas[1], new Rotation2d(odomAngles[1]))
        };
        m_sdkOdom.resetPosition(m_gyro.getRotation2d(), modulePositions, inverted);
    }

    // 
    // Drive
    //
    /**
     * @param chassisSpeeds the desired chassis speed.
     */
    public void setFieldRelativeSpeed(ChassisSpeeds chassisSpeeds) {
        chassisSpeed_pub[0] = chassisSpeeds.vxMetersPerSecond;
        chassisSpeed_pub[1] = chassisSpeeds.vyMetersPerSecond;
        chassisSpeed_pub[2] = chassisSpeeds.omegaRadiansPerSecond;
        
        targetModuleStates = m_kinematics.getComputedModuleStates(chassisSpeeds);
    }

    public void setRobotRelativeSpeed(ChassisSpeeds speeds) {
        SwerveModuleState[] mod_states = m_skdKine.toWheelSpeeds(speeds);
        targetModuleStates = m_kinematics.chassisSpeedToCustom(mod_states);
    }

    public ChassisSpeeds getRobotRelativeSpeed() {
        SwerveModuleState[] sdkModStates = m_kinematics.customToChassisSpeed(targetModuleStates);
        return m_skdKine.toChassisSpeeds(sdkModStates);

    }

    public void resetDrive() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].resetDriveAngleEncoder();
        }
    }

    public double smallestAngle(double largeAngle) {
        if(largeAngle > 0) {
            return largeAngle - Math.floor(Math.abs(largeAngle)/(2*Math.PI)) * (2*Math.PI);
        } else {
            return (largeAngle + Math.floor(Math.abs(largeAngle)/(2*Math.PI)) * (2*Math.PI)) + (2*Math.PI);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = (((moduleGroup[i].integratedDriveEncoder.getPosition() - encoderDriveOffset[i])/6.12) * (0.102*Math.PI));// - odomPrevDeltas[i];
            odomAngles[i] = smallestAngle(moduleGroup[i].getRadians());
        }

        // m_sdkOdom.update(m_gyro.getRotation2d(), new SwerveModulePosition[] {
        //         new SwerveModulePosition(Math.abs(odomDeltas[2]), new Rotation2d(odomAngles[2])),
        //         new SwerveModulePosition(Math.abs(odomDeltas[0]), new Rotation2d(odomAngles[0])),
        //         new SwerveModulePosition(Math.abs(odomDeltas[3]), new Rotation2d(odomAngles[3])),
        //         new SwerveModulePosition(Math.abs(odomDeltas[1]), new Rotation2d(odomAngles[1]))
        //     });
    
        m_sdkOdom.update(m_gyro.getRotation2d(), new SwerveModulePosition[] {
                new SwerveModulePosition(Math.abs(odomDeltas[0]), new Rotation2d(odomAngles[0])),
                new SwerveModulePosition(Math.abs(odomDeltas[1]), new Rotation2d(odomAngles[1])),
                new SwerveModulePosition(Math.abs(odomDeltas[2]), new Rotation2d(odomAngles[2])),
                new SwerveModulePosition(Math.abs(odomDeltas[3]), new Rotation2d(odomAngles[3])),
        });
    }

    //
    // Subsystems
    //
    public Gyro getGyro() {
        return m_gyro;
    } 

    public Odometry getOdometry() {
        return m_odom;
    }
}

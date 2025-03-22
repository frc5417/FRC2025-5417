// Copyright (c) FIRST and other WPILib contributors. test
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class FieldConstants {
      public static final double length = Units.feetToMeters(54);
      public static final double width = Units.feetToMeters(27);
      public static final double speakerHeightOptimal = 2.0;
      
      public static final double speakerExtensionOptimal = 0.15;
      public static final double pivotHeight = 0.381; //FIX THIS BUFFOON
  
      public static final Pose2d blueCenterNotePose = new Pose2d(3.5, 5.5, new Rotation2d());
      public static final Pose2d redCenterNotePose = new Pose2d(14.5, 5.5, Rotation2d.fromDegrees(-180));
    } 

  public static class OperatorConstants {
    public static final int kDriverPort = 0;
    public static final int kManipulatorPort = 1;
    public static final boolean fieldCentric = true; // front is the side with the battery
    public static final double joystickDeadband = 0.1; // HAS TO BE TUNED A BIT
  }
  
  public static class DriveBaseConstants {
    public static final double driveBaseRadius = (Double) 0.33 * Math.sqrt(2); 
  }

  public static class MotorConstants {
    public static final int kNeo550CL = 20; // current limit for neo 550s
    public static final int kNeoCL = 50; // current limit for Neos
    public static final int kVortexCL = 60; // current limit for Vortexes
  }

  public static class Swerve {
    public static final Double angularPercentage = 0.7;
    public static final Double XPercentage = 1.0;
    public static final Double YPercentage = 1.0;

    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kCoast;
    public static final double maxVelocity = 3.8; // m/s
    public static final double maxAngularVelocity = 10; //rad/sec
    public static final double maxModuleSpeed = 0.3;
    public static final boolean blueFlipState = false;

    public static final double angleKP = 0.055;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0013;
    public static final double angleKF = 0;
    public static final double voltageComp = 12;
  }

  public static class ModuleConstants {
    // 0 indexing
    public static final Integer[] driveMotorIDS = {11, 21, 31, 41}; 
    public static final Integer[] angleMotorIDS = {12, 22, 32, 42};
    public static final Integer[] CANCoderID = {13, 23, 33, 43};
    public static final String[] ModulePosition = {"Front Left", "Front Right", "Back Left", "Back Right"};
    // public static final Double[] motorDegrees = {93.69144, 88.68168, 93.16404, 85.69332};
    public static final Double[] motorDegrees = {0.0, 0.0, 0.0, 0.0};
    public static final Double degTolerance = 0.75;
    public static final boolean[] invertedMotors = {false, false, false, false};

  }

  public static class AlgaeConstants {
    public static final int algaeParentId = 50;
    public static final int algaeChildId = 51;
    public static final int algaeLimitValue = 0;
    public static final double algaePercentage = .7;
    public static final boolean algaeChildInversion = true;

  }
  
  public static class CoralConstants {
    public static final int coralWrist = 52;
    public static final int coralWheel = 53;
    public static final double coralWheelPercent = 0.5;
    public static final double coralWristPercent = 0.4;
  
    // PID
    public static final double coralWristP = 0;
    public static final double coralWristI = 0;
    public static final double coralWristD = 0;
    public static final double coralWristF = 0;
    
    // Set Position Values
    public static final double coralWristL2 = 0;
    public static final double coralWristL3 = 0;
    public static final double coralWristSource = 0;
  }

  public static class IntakeConstants {
    public static final int intakeParentId = 56;
    public static final int intakeChildId = 57; 
    public static final int intakeWristId = 53;

    public static final double intakeWheelPercent = 0.5;
    public static final double intakeJointPercent = 0.5;
    public static final boolean intakeChildInversion = true;

    // PID
    public static final double intakekP = 0.1;
    public static final double intakekI = 0;
    public static final double intakekD = 0;
    public static final double intakekF = 0;

    // Set Position Value
    public static final double intakeMin = 2;
    public static final double intakeMax = 43;
    public static final double intakeIntake = 2; // Intaking from source
    public static final double intakeReef = 15; // Outtaking to reef
    public static final double intakeL1 = 0; // Outtaking to L1
  }

  public static class ElevatorConstants {
    public static final double elevatorMin = 0.75;
    public static final double elevatorMax = 86;
    public static final int elevatorParentId = 55;
    public static final int elevatorChildId = 52;
    public static final boolean elevatorChildInvert = true;

    public static final double feedKS = 0.55;
    public static final double feedKV = 0.13;
    public static final double feedKA = 0;

    // PID 
    public static final double elevatorkP = 0.4; 
    public static final double elevatorkI = 0;
    public static final double elevatorkD = 0;
    public static final double elevatorkF = 0;

    // Set Position Values
    public static final double elevatorSource = 1.5;
    public static final double elevatorL1 = 10; // guestimate
    public static final double elevatorL2 = 46;
    public static final double elevatorL3 = 73;
  }

  public static class ClimbConstants {
    public static final int climbMotorId = 8;
    public static final double climbPercent = 0.25;
    public static final double climbMin = 1.5;
    public static final double climbMax = 23;

    // PID 
    public static final double climbkP = 0;
    public static final double climbkI = 0;
    public static final double climbkD = 0;
    public static final double climbkF = 0;  
  }

  public static class Auton {
    public static final PIDController X_Pos = new PIDController(1.3, 0, 0);
    public static final PIDController Y_Pos = new PIDController(1.3, 0, 0);
    public static final PIDController Theta_Pos = new PIDController(0.01, 0.0, 0.0); //0.5 p 0.15 0.035
    public static final double speedClamp = 0.4;
    public static final double speedRotClamp = 0.3;
    public static final double poseTolerance = 0.1;
    public static final double thetaTolerance = 3; //deg
    public static final Double[] robot_size = {0.66, 0.66};
    public static final Double[] field_size = {8.2, 16.0};
    public static final Double[] BlueObstacle_TopLeft = {2.5, 6.0};
    public static final Double[] BlueObstacle_BottomRight = {6.0,2.5};
    public static final Double[] RedObstacle_TopLeft = {2.5, 13.5};
    public static final Double[] RedObstacle_BottomRight = {6.0, 10.0};
  }

  public static class VisionConstants {
    // public static final Transform3d robotToCam =
    //         new Transform3d(
    //                new Translation3d(0.2794, 0.1524, 0.254),
    //                 new Rotation3d(
    //                         0, 
    //                         (66*(Math.PI/180)),
    //                         0)); 

    // camera is mounted facing backward, x is how far backward, y is how far left or right, z is how far high
    // CENTER AT THE CENTER OF THE DRIVETRAIN
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(-0.229, 0.254, 0.514), new Rotation3d(0,(75.0*(Math.PI/180.0)),(180.0*(Math.PI/180.0)))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  //   // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final String cameraName = "Limelight";
    public static final double maxDistanceAway = 2.0;
    public static final double forwardKP = 0.1;
    public static final double forwardToAngleRatio = 0.5;
    
    public static final double CAMERA_HEIGHT_METERS = 1;
    public static final double TARGET_HEIGHT_METERS = 1.5;
    public static final double CAMERA_PITCH_RADIANS = 0;
  }
}

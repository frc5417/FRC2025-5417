// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //
  // General Constants
  //

  /**
   * Constants which are utilized for general-hardware constants.
   */
  public static class HardwareConstants {
    public static final int kNeo550CL = 20; // current limit for NEO 550s
    public static final int kNeoCL = 50; // current limit for NEOs
    public static final int kVortexCL = 60; // current limit for Vortexes
  }

  /**
   * Constants which consist of controller-adjacent variables.
   */
  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final double joystickDeadband = 0.1;
  }

  //
  // Drive Train Constants
  //

  /**
   * Constants which emcompass all the constant variables needed for the
   * drive base.
   */
  public static class DriveBaseConstants {
    // Gyro
    public static final int pigeonID = 59;

    // Kinematics
    public static final boolean kIsFieldCentric = true;

    // Module
    public static final Double angularPercentage = 0.7;
    public static final Double XPercentage = 1.0;
    public static final Double YPercentage = 1.0;

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

  /**
   * Constants that relate to individual modules or 'global' variables (consistent constants between modules).
   */
  public static class ModuleConstants {
    // Module-Specific variables
    public static final int[] driveMotorIDs = {11, 21, 31, 41};
    public static final boolean[] invertedDrive = {true, true, true, true};
    public static final int[] angleMotorIDs = {12, 22, 32, 42};
    public static final int[] angleOffset = {0, 0, 0, 0}; // offset for angle motors by degrees
    public static final int[] cancoderIDs = {13, 23, 33, 43};
    public static final String[] modulePosition = {"Front Left", "Front Right", "Back Left", "Back Right"};

    public static final double degTolerance = 0.75;

    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kCoast;

    public static final double kP = 0.50;
    public static final double kI = 0;
    public static final double kD = 0.013;
  }

  //
  // Subsystem Constants
  //

  /**
   * Constants which relate to the elevator subsystem.
   */
  public static class ElevatorConstants {
    // Hardware + Mechanical
    public static final int elevatorParentId = 55;
    public static final int elevatorChildId = 52;
    public static final boolean elevatorChildInvert = true;
    public static final double shaftRadius = 0.0127; // .5 inches -> m
    public static final double kElevatorPercentage = 1.5;

    // Feedforward
    public static final double feedKS = 0.55;
    public static final double feedKV = 0.13;
    public static final double feedKA = 0;

    // PID 
    public static final double elevatorkP = 0.2; //0.4 
    public static final double elevatorkI = 0;
    public static final double elevatorkD = 0;
    public static final double elevatorkF = 0;

    // Set Position Values
    public static final double elevatorMin = 0.75;
    public static final double elevatorMax = 105;
    public static final double elevatorSource = 0.75; //change
    public static final double elevatorL1 = 10; // guestimate
    public static final double elevatorL2 = 46; // lower
    public static final double elevatorL3 = 73; // lower
    public static final double elevatorL4 = 103.7;
  }

  /**
   * Constants which relate to the the coral intake subsystem.
   */
  public static class CoralConstants {
    // Hardware
    public static final int intakeID = 51;
    public static final int sensorID = 0; // change
    public static final double kCoralPercent = 0.5;
  }
}

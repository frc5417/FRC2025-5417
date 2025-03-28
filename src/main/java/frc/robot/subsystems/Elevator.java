// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//44eeimport edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The subsystem for controlling the elevator on the robot.
 */
public class Elevator extends SubsystemBase {
  private final SparkMax elevatorParent;
  private final SparkMax elevatorChild;
  public final RelativeEncoder elevatorParentEncoder;
  public final RelativeEncoder elevatorChildEncoder;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ElevatorConstants.feedKS, 
                                                   Constants.ElevatorConstants.feedKV, Constants.ElevatorConstants.feedKA);
  private SparkClosedLoopController elevatorPID;

  public Elevator() {
    elevatorParent = new SparkMax(Constants.ElevatorConstants.elevatorParentId, MotorType.kBrushless);
    elevatorChild = new SparkMax(Constants.ElevatorConstants.elevatorChildId, MotorType.kBrushless);

    motorConfig();
    elevatorPID = elevatorParent.getClosedLoopController();

    elevatorParentEncoder = elevatorParent.getEncoder();
    elevatorChildEncoder = elevatorChild.getEncoder();
  }

  //
  // Hardware
  //
  
  /**
  * Configures the elevator motors.
  */
  private void motorConfig() {
    SparkFlexConfig parentConfig = new SparkFlexConfig();
    SparkFlexConfig childConfig = new SparkFlexConfig();

    parentConfig.idleMode(IdleMode.kBrake);
    parentConfig.smartCurrentLimit(Constants.HardwareConstants.kVortexCL);
    parentConfig.closedLoop.pidf(Constants.ElevatorConstants.elevatorkP, Constants.ElevatorConstants.elevatorkI, 
        Constants.ElevatorConstants.elevatorkD, Constants.ElevatorConstants.elevatorkF);
    elevatorParent.configure(parentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    childConfig.apply(parentConfig);
    childConfig.follow(elevatorParent, Constants.ElevatorConstants.elevatorChildInvert);
    elevatorChild.configure(childConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //
  // Methods to Control the Elevator
  //

  /**
   * Runs the elevator through a feedforward loop.
   * @param power
   */
  public void setElevatorPower(double power) {
    elevatorParent.setVoltage(feedforward.calculate(.35 * power));
    elevatorParent.set(power);
  }

  /**
   * Sets the elevator to a given position.  Utilizes PID.
   * @param pos position as revolutions
   */
  public void setElevatorPos(double pos) {
    elevatorPID.setReference(pos, ControlType.kPosition);
  }
    
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Revs", getEncoder());
    SmartDashboard.putNumber("Elevator Velocity", getLinearVelocity());
  }

  //
  // Data-Related Methods
  //

  /**
   * Position is under the units of revolutions.
   * @return revolutions of the elevator motor
   */
  public double getEncoder() {
    return elevatorParentEncoder.getPosition();
  }

  /**
   * Velocity in meters / second.
   * @return the linear velocity of the elevator.
   */
  public double getLinearVelocity() {
    double omega = (elevatorParentEncoder.getVelocity() * 2 * Math.PI) / 60; // conversion from RPM -> ms^-1
    return Constants.ElevatorConstants.shaftRadius * omega; // rotational kinematics -> linear
  }
}

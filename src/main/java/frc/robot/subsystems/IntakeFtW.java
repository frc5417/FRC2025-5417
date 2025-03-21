// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeFtW extends SubsystemBase {
  /** Creates a new IntakeFtW. */
    private final SparkMax intakeParent;
    private final SparkMax intakeChild;
    private final SparkMax intakeWrist;
    private SparkClosedLoopController intakeWristPID;
    private final RelativeEncoder intakeEncoder;
    // private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ElevatorConstants.feedKS, 
    //                                              Constants.ElevatorConstants.feedKV, Constants.ElevatorConstants.feedKA);

  public IntakeFtW() {
    intakeParent = new SparkMax(Constants.IntakeConstants.intakeParentId, MotorType.kBrushless);
    intakeChild = new SparkMax(Constants.IntakeConstants.intakeChildId, MotorType.kBrushless);
    intakeWrist = new SparkMax(Constants.IntakeConstants.intakeWristId, MotorType.kBrushless);

    configMotors();
    intakeWristPID = intakeWrist.getClosedLoopController();

    intakeEncoder = intakeWrist.getEncoder();
  }

  public void setIntakeWheelPower(double power) {
    // intakeParent.setVoltage(feedforward.calculate(.5 * power));
    // intakeChild.setVoltage(feedforward.calculate(.5 * power));
    intakeParent.set(power);
    intakeChild.set(power);
  }

  public void setIntakeWristPower(double power) {
    intakeWrist.set(power);
  }

  public void setIntakeWristPos(double pos) {
    intakeWristPID.setReference(pos, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Algae Switch", getAlgaeSwitch());
    SmartDashboard.putNumber("Intake Wrist", getEncoder());
  }

  public double getEncoder() {
    return intakeEncoder.getPosition();
  }

  private void configMotors() {
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    
    wristConfig.smartCurrentLimit(Constants.MotorConstants.kNeoCL);
    wristConfig.closedLoop.pidf(Constants.IntakeConstants.intakekP,Constants.IntakeConstants.intakekI,
      Constants.IntakeConstants.intakekD, Constants.IntakeConstants.intakekF);
    intakeWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    // SparkMaxConfig parentConfig = new SparkMaxConfig();
    // SparkMaxConfig childConfig = new SparkMaxConfig();

  }
}
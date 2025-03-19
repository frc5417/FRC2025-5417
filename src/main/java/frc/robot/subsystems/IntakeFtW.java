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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeFtW extends SubsystemBase {
  /** Creates a new IntakeFtW. */
    private final SparkMax intakeParent;
    private final SparkMax intakeChild;
    private final SparkMax intakeWrist;
    private SparkClosedLoopController intakeWristPID;
    private final RelativeEncoder intakeEncoder;

  public IntakeFtW() {
    intakeParent = new SparkMax(Constants.IntakeConstants.intakeParentId, MotorType.kBrushless);
    intakeChild = new SparkMax(Constants.IntakeConstants.intakeChildId, MotorType.kBrushless);
    intakeWrist = new SparkMax(Constants.IntakeConstants.intakeWristId, MotorType.kBrushless);

    intakeEncoder = intakeWrist.getEncoder();
    intakeWristPID = intakeWrist.getClosedLoopController();
    
    configMotors();
    //intakeWristPID = intakeWrist.getClosedLoopController();
  }

  public void setIntakeWheelPower(double power) {
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
    SparkMaxConfig parentConfig = new SparkMaxConfig();
    SparkMaxConfig childConfig = new SparkMaxConfig();

    parentConfig.idleMode(IdleMode.kBrake);
    parentConfig.smartCurrentLimit(Constants.MotorConstants.kNeo550CL);

    childConfig.apply(parentConfig);
    childConfig.follow(intakeParent, Constants.IntakeConstants.intakeChildInversion);
    // childConfig.inverted(Constants.ManipulatorConstants.intakeChildInversion);
    
    intakeParent.configure(parentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeChild.configure(childConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  
}
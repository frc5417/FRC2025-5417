// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private SparkMax climbMotor;

  /** Creates a new Climb. */
  public Climb() {
    climbMotor = new SparkMax(Constants.ClimbConstants.climbMotorId, MotorType.kBrushless);

    configMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runClimb(double power) {
    climbMotor.set(power);
  }

  private void configMotor() {
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(Constants.MotorConstants.kNeoCL);
    climbConfig.idleMode(IdleMode.kBrake);
    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
  }
}

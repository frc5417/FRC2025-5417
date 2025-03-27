// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
  private final SparkMax intakeMotor;

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    intakeMotor = new SparkMax(Constants.CoralConstants.intakeID, MotorType.kBrushless);   
    motorConfig(); 
  }

  public void setPower(double power) {
    intakeMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void motorConfig() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(Constants.HardwareConstants.kNeoCL);

    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

}

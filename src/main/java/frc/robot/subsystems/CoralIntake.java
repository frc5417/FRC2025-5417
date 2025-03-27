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

//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
    private final SparkMax coralWrist;
    private final SparkMax coralWheel;
    private SparkClosedLoopController coralWristPID;
    private final RelativeEncoder wristEncoder;

    //private DigitalInput coralIntakeSwitch = new DigitalInput(Constants.ManipulatorConstants.coralIntakeLimitValue);

  public CoralIntake() {
    coralWrist = new SparkMax(Constants.CoralConstants.coralWrist, MotorType.kBrushless);
    coralWheel = new SparkMax(Constants.CoralConstants.coralWheel, MotorType.kBrushless);

    wristEncoder = coralWrist.getEncoder();
    coralWristPID = coralWrist.getClosedLoopController();

    configMotors();
    coralWristPID = coralWrist.getClosedLoopController();
  }

  // public void setCoralWristPower(double power) {
  //   // coralWrist.set(power);
  //   wristPID.setReference(power, ControlType.kDutyCycle);
  // }

  public void setCoralWheelPower(double power) {
    coralWheel.set(power);
  }

  public void setCoralWristPos(double pos) {
    coralWristPID.setReference(pos, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Wrist", getEncoder());
  }

  private double getEncoder() {
    return wristEncoder.getPosition();
  }

  private void configMotors() {
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    SparkMaxConfig wheelConfig = new SparkMaxConfig();

    wristConfig.idleMode(IdleMode.kBrake);
    wheelConfig.idleMode(IdleMode.kBrake);

    wristConfig.smartCurrentLimit(Constants.MotorConstants.kNeoCL);
    wheelConfig.smartCurrentLimit(Constants.MotorConstants.kNeo550CL);

    // wristConfig.closedLoop.pid(Constants.ManipulatorConstants.coralWristkP, Constants.ManipulatorConstants.coralWristkI, Constants.ManipulatorConstants.coralWristkD);

    coralWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralWheel.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
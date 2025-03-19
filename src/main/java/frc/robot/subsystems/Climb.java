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

public class Climb extends SubsystemBase {
  private final SparkMax climbMotor;
  public final RelativeEncoder climbEncoder;
  private SparkClosedLoopController climbPID;
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ElevatorConstants.feedKS, 
  Constants.ElevatorConstants.feedKV, Constants.ElevatorConstants.feedKA);


  /** Creates a new Climb. */
  public Climb() {
    climbMotor = new SparkMax(Constants.ClimbConstants.climbMotorId, MotorType.kBrushless);

    configMotor();
    climbPID = climbMotor.getClosedLoopController();

    climbEncoder = climbMotor.getEncoder();
  }

  public void setClimbPower(double power) {
    climbMotor.setVoltage(feedforward.calculate(.1*power));
    climbMotor.set(power);
  }

  public void setClimbPos(double pos) {
    climbPID.setReference(pos, ControlType.kPosition);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Encoder", climbEncoder.getPosition());
  }

  private void configMotor() {
    SparkMaxConfig climbConfig = new SparkMaxConfig();

    climbConfig.smartCurrentLimit(Constants.MotorConstants.kNeoCL);
    climbConfig.idleMode(IdleMode.kBrake);
    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
  }
}
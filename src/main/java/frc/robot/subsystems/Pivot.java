// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//44eeimport edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    public final RelativeEncoder pivotEncoder;
    private final SparkMax pivot;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ElevatorConstants.feedKS, 
                                                    Constants.ElevatorConstants.feedKV, Constants.ElevatorConstants.feedKA);
    //private SparkClosedLoopController elevatorPID;

    public Pivot() {
        pivot = new SparkMax(8, MotorType.kBrushless);
        pivotEncoder = pivot.getEncoder();
        motorConfig();
    }

    public void setPivotPower(double power) {
        pivot.setVoltage(feedforward.calculate(.5 * power));
        pivot.set(power);
        // elevatorChild.setVoltage(feedforward.calculate(.5 * -power));
        // elevatorChild.set(power);
    }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Pivot Encoder", pivotEncoder.getPosition());
    }

    /**
     * Configures the elevator motors.
     */
    private void motorConfig() {
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.smartCurrentLimit(Constants.MotorConstants.kVortexCL);
        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}

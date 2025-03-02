// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//44eeimport edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private final SparkMax pivot;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ElevatorConstants.feedKS, 
                                                    Constants.ElevatorConstants.feedKV, Constants.ElevatorConstants.feedKA);
    //private SparkClosedLoopController elevatorPID;

    public Pivot() {
        pivot = new SparkMax(Constants.ElevatorConstants.elevatorParentId, MotorType.kBrushless);

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
    }

    /**
     * Configures the elevator motors.
     */
    private void motorConfig() {
        SparkFlexConfig pivot = new SparkFlexConfig();

        pivot.idleMode(IdleMode.kBrake);
        pivot.smartCurrentLimit(Constants.MotorConstants.kVortexCL);
        //pivot.configure(pivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}

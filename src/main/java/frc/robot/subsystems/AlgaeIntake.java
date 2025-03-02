// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
    private final SparkMax algaeParent;
    private final SparkMax algaeChild;
    private final DigitalInput algaeIntakeSwitch = new DigitalInput(Constants.AlgaeConstants.algaeLimitValue);

  public AlgaeIntake() {
    algaeParent = new SparkMax(Constants.AlgaeConstants.algaeParentId, MotorType.kBrushless);
    algaeChild = new SparkMax(Constants.AlgaeConstants.algaeChildId, MotorType.kBrushless);
    
    configMotors();
  }

  public void setAlgaePower(double power) {
    // negative power intakes
    if (getAlgaeSwitch() && power < 0) {
      algaeParent.set(0);
      // algaeChild.set(0);
    }
    else {
      algaeParent.set(power);
      // algaeChild.set(power);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Algae Switch", getAlgaeSwitch());
  }

  private void configMotors() {
    SparkMaxConfig parentConfig = new SparkMaxConfig();
    SparkMaxConfig childConfig = new SparkMaxConfig();

    parentConfig.idleMode(IdleMode.kBrake);
    parentConfig.smartCurrentLimit(Constants.MotorConstants.kNeo550CL);
    algaeParent.configure(parentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    childConfig.apply(parentConfig);
    childConfig.follow(algaeParent, Constants.AlgaeConstants.algaeChildInversion);
    // childConfig.inverted(Constants.ManipulatorConstants.algaeChildInversion);
    algaeChild.configure(childConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public boolean getAlgaeSwitch() {
    return !algaeIntakeSwitch.get();
    // return false;
  }
}
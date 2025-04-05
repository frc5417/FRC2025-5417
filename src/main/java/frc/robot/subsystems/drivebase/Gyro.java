// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.CustomMathLib;
import frc.robot.Constants;

public class Gyro extends SubsystemBase {
  private final Pigeon2 m_pigeon;

  public Gyro() {
    m_pigeon = new Pigeon2(Constants.DriveBaseConstants.pigeonID, "canivore");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yaw", CustomMathLib.normalizeDegrees(getRotation2d().getDegrees()));
  }

  //
  // Hardware-related Methods
  //
  /**
   * Resets the yaw rotation of the gyro.
   */
  public void resetYaw() {
    m_pigeon.reset();
  }

  //
  // Get & Set Methods
  //
  public Rotation2d getRotation2d() {
    return m_pigeon.getRotation2d();
  }
}

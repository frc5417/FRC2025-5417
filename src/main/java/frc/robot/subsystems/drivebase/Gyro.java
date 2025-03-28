// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class Gyro {
  private final Pigeon2 m_pigeon;

  public Gyro() {
    m_pigeon = new Pigeon2(Constants.DriveBaseConstants.pigeonID, "canivore");
  }

  //
  // Manager Methods
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

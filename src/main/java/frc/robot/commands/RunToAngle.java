// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivebase.DriveBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunToAngle extends Command {
  //
  // Subsystems
  //
  private final DriveBase m_driveBase;

  //
  // Variables
  //
  private boolean terminate;
  private final double kDesiredDegree;

  /** Creates a new RunToAngle. */
  public RunToAngle(DriveBase drivebase, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveBase = drivebase;
    kDesiredDegree = degrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = kDesiredDegree - m_driveBase.getGyro().getRotation2d().getDegrees();

    if (Math.abs(error) < Constants.DriveBaseConstants.kErrorTolerance) {
      // yaw is acceptable
      terminate = true;
    } else {
      // yaw is off

      /*
       * ratio of how many radians are needed to correct to the desired angle
       */
      double omega = (error / 180) * (Math.PI / 2);
      m_driveBase.setRobotRelativeSpeed(new ChassisSpeeds(0, 0, omega));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.setRobotRelativeSpeed(new ChassisSpeeds(0, 0, 0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}

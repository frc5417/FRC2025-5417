// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.DriveBase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunDriveBaseRobot extends Command {
  /* Subsystems */
  private final DriveBase m_driveBase;

  /* Variables */
  private final ChassisSpeeds speeds;
  private boolean terminate;

  /** Creates a new RunDriveBase. */
  public RunDriveBaseRobot(DriveBase driveBase, double x, double y, double omega) {
    /* Subsystems */
    m_driveBase = driveBase;

    /* Variables */
    speeds = new ChassisSpeeds(x, y, omega);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveBase.setRobotRelativeSpeed(speeds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.setFieldRelativeSpeed(new ChassisSpeeds(0,0,0));
    terminate = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivebase.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveAuto extends SequentialCommandGroup {
  /* Subsystems */
  private final DriveBase m_driveBase;

  
  public LeaveAuto(DriveBase driveBase) {
    /* Subsystems */
    m_driveBase = driveBase;

    /* Command Sequence */
    addCommands(
      new InstantCommand(() -> 
          m_driveBase.setFieldRelativeSpeed(new ChassisSpeeds(-.2,0,0))
      ),
      new WaitCommand(4),
      new InstantCommand(() ->
          m_driveBase.setFieldRelativeSpeed(new ChassisSpeeds(0,0,0))
      )
    );
  }
}

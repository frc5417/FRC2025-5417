// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.RunCoral;
import frc.robot.commands.RunElevator;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drivebase.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4Auton extends SequentialCommandGroup {
  /* Subsystems */
  private final DriveBase m_driveBase;
  private final Elevator m_elevator;
  private final CoralIntake m_coral;

  public L4Auton(DriveBase driveBase, Elevator elevator, CoralIntake coral) {
    /* Subsystems */
    m_driveBase = driveBase;
    m_elevator = elevator;
    m_coral = coral;

    /* Command Sequence */
    addCommands(
      // Leave barge
      new InstantCommand(() -> 
          m_driveBase.setFieldRelativeSpeed(new ChassisSpeeds(-.2, 0, 0))
      ),
      new WaitCommand(2.5),

      // go back from coral
      new InstantCommand(() ->
          m_driveBase.setFieldRelativeSpeed(new ChassisSpeeds(.1, 0, 0))
      ),
      new WaitCommand(.5),

      // Stop + L4
      new InstantCommand(() ->
        m_driveBase.setFieldRelativeSpeed(new ChassisSpeeds(0, 0, 0))
      ),
      new RunElevator(m_elevator, Constants.ElevatorConstants.elevatorL4),
      new RunCoral(m_coral, -0.5).withTimeout(2),
      new RunElevator(m_elevator, Constants.ElevatorConstants.elevatorMin)
    );
  }
}

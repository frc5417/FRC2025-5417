// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.

  private final DriveBase m_driveBase;
  // private final AlgaeIntake m_algae;
  // private final CoralIntake m_coral;
  // private final Elevator m_elevator;
  private final Vision m_vision;

  // AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // update to 2025
  

  double prev_omega = 0;
  double prev_xVel = 0;
  double prev_yVel = 0;
 
  int counter = 0;
  int timer = 0;
  int lastTime = 0;

  double wristPos = 0.0;

  double manipulatorPosition = 0;

  public TeleopDrive(DriveBase driveBase, Vision vision) {
    m_driveBase = driveBase;
    // m_algae = algaeIntake;
    // m_coral = coralIntake;
    // m_elevator = elevator;
    m_vision = vision;
  }

  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //
    // Vision
    //

    //
    // Swerve Drive
    //
    double xVel = (-RobotContainer.getDriverRightJoyY() * 0.90) + (prev_xVel * 0.10); // originally .9 and .1
    double yVel = (-RobotContainer.getDriverRightJoyX() * 0.90) + (prev_yVel * 0.10); // originally .9 and 1.
    double omega = (-RobotContainer.getDriverLeftJoyX() * 0.90) + (prev_omega * 0.10);

    prev_xVel = xVel;
    prev_yVel = yVel;
    prev_omega = omega;
    
    m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(xVel, yVel, omega));

    // Odometery
    if (RobotContainer.getDriveABool()) {
      m_driveBase.resetYaw();
    }

    // //
    // // Algae Intake
    // //
    // double algaePower =  RobotContainer.getManipulatorLeftTrigger() - RobotContainer.getManipulatorRightTrigger();
    // m_algae.setAlgaePower(algaePower * Constants.ManipulatorConstants.algaePercentage);

    // //
    // // Coral Intake
    // //
    // double coralPower = 0;
    // if (RobotContainer.getManipulatorRightBumperBool()) {
    //   coralPower++;
    // }
    // if (RobotContainer.getManipulatorLeftBumperBool()) {
    //   coralPower--;
    // }
    // // m_coral.setCoralWheelPower(coralPower);
    // m_coral.setCoralWheelPower(coralPower * Constants.ManipulatorConstants.coralWheelPercent);
    // m_coral.setCoralWristPower(RobotContainer.getManipulatorLeftJoyY() * Constants.ManipulatorConstants.coralWristPercent);

    // //
    // // Elevator
    // //
    // m_elevator.setElevatorPower(-RobotContainer.getManipulatorRightJoyY()); // - is up, + is down

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.resetDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}

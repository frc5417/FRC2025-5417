// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Controllers;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivebase.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleOpDrive extends Command {
  private DriveBase m_driveBase;
  private Elevator m_elevator;
  private CoralIntake m_coral; // negative to intake

  //
  // DriveBase Variables
  //
  double prev_xVel = 0;
  double prev_yVel = 0;
  double prev_omega = 0;

  // 
  // Elevator Variables
  //
  double elevatorPos = 0;

  /** Creates a new TeleOpDrive. */
  public TeleOpDrive(DriveBase driveBase, Elevator elevator, CoralIntake coralIntake) {
    m_driveBase = driveBase;
    m_elevator = elevator;
    m_coral = coralIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //
    // DriveBase
    //
    double xVel = (-Controllers.DriverInput.getRightJoyY() * 0.90) + (prev_xVel * 0.10); 
    double yVel = (-Controllers.DriverInput.getRightJoyX() * 0.90) + (prev_yVel * 0.10); 
    double omega = (-Controllers.DriverInput.getLeftJoyX() * 0.90) + (prev_omega * 0.10); 

    prev_xVel = xVel;
    prev_yVel = yVel;
    prev_omega = omega;

    m_driveBase.setFieldRelativeSpeeds(new ChassisSpeeds(xVel, yVel, omega));

    // Odometry
    if (Controllers.DriverInput.getA()) {
      m_driveBase.getGyro().resetYaw();
    }

    //
    // Elevator
    //
    elevatorPos += Constants.ElevatorConstants.kElevatorPercentage * -Controllers.ManipulatorInput.getRightJoyY();
    elevatorPos = MathUtil.clamp(elevatorPos, Constants.ElevatorConstants.elevatorMin, Constants.ElevatorConstants.elevatorMax);
    m_elevator.setElevatorPos(elevatorPos);
    SmartDashboard.putNumber("Desired Elevator Pos", elevatorPos);

    // Set Positions
    if (Controllers.ManipulatorInput.getX()) { // L1
      // elevatorPos = Constants.ElevatorConstants.elevatorL1;
      m_elevator.setElevatorPos(Constants.ElevatorConstants.elevatorL1);
    }
    if (Controllers.ManipulatorInput.getY()) { // L2
      // elevatorPos = Constants.ElevatorConstants.elevatorL2;
      m_elevator.setElevatorPos(Constants.ElevatorConstants.elevatorL2);
    }
    if (Controllers.ManipulatorInput.getB()) { // L3
      // elevatorPos = Constants.ElevatorConstants.elevatorL3;
      m_elevator.setElevatorPos(Constants.ElevatorConstants.elevatorL3);
    } 
    if (Controllers.ManipulatorInput.getA()) { // L4
      // elevatorPos = Constants.ElevatorConstants.elevatorL4;
      m_elevator.setElevatorPos(Constants.ElevatorConstants.elevatorL4);
    }
    if (Controllers.ManipulatorInput.getDpadRight()) { // Source
      // elevatorPos = Constants.ElevatorConstants.elevatorSource;
      m_elevator.setElevatorPos(Constants.ElevatorConstants.elevatorSource);
    }

    //
    // Coral Intake
    //
    double coralPower = Controllers.ManipulatorInput.getLeftTrigger() - Controllers.ManipulatorInput.getRightTrigger();
    m_coral.setPower(Constants.CoralConstants.kCoralPercent * coralPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



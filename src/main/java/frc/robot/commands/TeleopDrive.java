// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private final AlgaeIntake m_algae;
  // private final CoralIntake m_coral;
  private final Elevator m_elevator;
  private final IntakeFtW m_intake;
  //private final Climb m_climb;
  private final Vision m_vision;
<<<<<<< HEAD
  private final Pivot m_pivot;
=======
  public final static CommandXboxController m_manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorPort);
>>>>>>> 6b034d3c436c64979ceba36b778100ffe4df5a9c

  // AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // update to 2025
  
  double prev_omega = 0;
  double prev_xVel = 0;
  double prev_yVel = 0;
 
  int counter = 0;
  int timer = 0;
  int lastTime = 0;

  double wristPos = 0.0;
  double elevatorPos = 0;
  double manipulatorPosition = 0;

<<<<<<< HEAD
  public TeleopDrive(DriveBase driveBase, AlgaeIntake algae, Vision vision, Pivot pivot) {
=======
  public TeleopDrive(DriveBase driveBase, IntakeFtW intake, Elevator elevator, Vision vision) {
>>>>>>> 6b034d3c436c64979ceba36b778100ffe4df5a9c
    m_driveBase = driveBase;
    m_algae = algae;
    // m_coral = coralIntake;
    m_elevator = elevator;
    m_intake = intake;
    //m_climb = climb;
    m_vision = vision;
    m_pivot = pivot;
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
    if (RobotContainer.getDriveYBool()) {
      m_driveBase.resetYaw();
<<<<<<< HEAD
    }
      
    // 85.2
    m_pivot.setPivotPower(RobotContainer.getManipulatorRightJoyY() * .5);
    //
    // Algae Intake
    //
    double algaePower = RobotContainer.getManipulatorLeftTrigger() - RobotContainer.getManipulatorRightTrigger();
    m_algae.setAlgaePower(algaePower * Constants.AlgaeConstants.algaePercentage);
=======
    }    
>>>>>>> 6b034d3c436c64979ceba36b778100ffe4df5a9c

    //
    // New Fort Worth Intake
    //
    double intakePower =  RobotContainer.getManipulatorLeftTrigger() - RobotContainer.getManipulatorRightTrigger();
    m_intake.setIntakeWheelPower(Constants.IntakeConstants.intakeWheelPercent * intakePower);
    m_intake.setIntakeWristPower(Constants.IntakeConstants.intakeJointPercent * RobotContainer.getManipulatorLeftJoyY());

    // wristPos += RobotContainer.getManipulatorLeftJoyY();
    // wristPos = MathUtil.clamp(wristPos, Constants.IntakeConstants.intakeMin, Constants.IntakeConstants.intakeMax);
    // m_intake.setIntakeWristPos(wristPos);

    // 
    // Climb
    //
    // double climbPower = 0.0;
    // if (RobotContainer.getManipulatorRightBumperBool()) {
    //   climbPower++;
    // } 
    // if (RobotContainer.getManipulatorLeftBumperBool()) {
    //   climbPower--;
    // }
    // climbPos = m_climb.getEncoder();
    // climbPos = MathUtil.clamp(climbPos, Constants.ClimbConstants.climbMin, Constants.ClimbConstants.climbMax);
    // m_climb.setClimbPos(climbPos);
    // m_climb.setClimbPower(Constants.ClimbConstants.climbPercent * climbPower);

    //
    // Elevator
    //
    elevatorPos += -RobotContainer.getManipulatorRightJoyY();
    elevatorPos = MathUtil.clamp(elevatorPos, Constants.ElevatorConstants.elevatorMin, Constants.ElevatorConstants.elevatorMax);
    m_elevator.setElevatorPos(elevatorPos);
    //m_elevator.setElevatorPower(-RobotContainer.getManipulatorRightJoyY()); // - is up, + is down
    
    //
    // Macros
    //
    if (RobotContainer.getManipulatorXBool()) { // L1
      // wristPos = Constants.IntakeConstants.intakeL1;
      // elevatorPos = Constants.ElevatorConstants.elevatorL1;
      m_elevator.setElevatorPos(Constants.ElevatorConstants.elevatorL1);
    }
    if (RobotContainer.getManipulatorBBool()) { // L2
      // wristPos = Constants.IntakeConstants.intakeReef;
      // elevatorPos = Constants.ElevatorConstants.elevatorL2;
      m_elevator.setElevatorPos(Constants.ElevatorConstants.elevatorL2);
    }
    if (RobotContainer.getManipulatorYBool()) { // L3
      // wristPos = Constants.IntakeConstants.intakeReef;
      // elevatorPos = Constants.ElevatorConstants.elevatorL3;
      m_elevator.setElevatorPos(Constants.ElevatorConstants.elevatorL3);
    } 
    if (RobotContainer.getManipulatorABool()) { // Source
      // wristPos = Constants.IntakeConstants.intakeSource;
      // elevatorPos = Constants.ElevatorConstants.elevatorSource;
      m_elevator.setElevatorPos(Constants.ElevatorConstants.elevatorSource);
    }
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

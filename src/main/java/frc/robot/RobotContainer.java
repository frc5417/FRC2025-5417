// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.autos.AutonSelect;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivebase.*;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //
  // Subsystems
  //
  private static Gyro m_gyro = new Gyro();
  private static DriveBase m_driveBase = new DriveBase(m_gyro);

  private static Elevator m_elevator = new Elevator();
  private static CoralIntake m_coralIntake = new CoralIntake();

  public static AutonSelect m_autonSelect = new AutonSelect();
  public static TeleOpDrive teleOpDrive = new TeleOpDrive(m_driveBase, m_elevator, m_coralIntake);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    registerNamedCommands();

    m_gyro.resetYaw();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Registers commands for use in PathPlanner.
   */
  private void registerNamedCommands() {
    // NamedCommands.registerCommand(null, teleOpDrive);
    NamedCommands.registerCommand("Run Coral", new RunCoral(m_coralIntake, Constants.CoralConstants.kCoralPercent).withTimeout(3));
    NamedCommands.registerCommand("Run Elevator Minimum", new RunElevator(m_elevator, Constants.ElevatorConstants.elevatorMin).withTimeout(2));
    NamedCommands.registerCommand("Run Elevator L4", new RunElevator(m_elevator, Constants.ElevatorConstants.elevatorL4).withTimeout(2));
  }

  //
  // Robot Control
  //

  public void beginTeleOp() {
    teleOpDrive.schedule();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return m_autonSelect.getAuto();
    return Commands.none();
  }
}

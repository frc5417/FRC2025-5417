// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunCoralWrist extends Command {
  private double power;
  private CoralIntake m_coral;
  private boolean terminate = false;

  /** Creates a new RunCoral. */
  public RunCoralWrist(CoralIntake coralIntake) {
    //coralWheel = intake;
    m_coral = coralIntake;
    this.power = 0;
    //public final static CommandXboxController m_manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorPort);

    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.power = RobotContainer.getManipulatorLeftJoyX();
    SmartDashboard.putNumber("Wrist Power", this.power);
    // m_coral.setCoralWristPower(this.power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_coral.setCoralWristPower(0);
    terminate = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}
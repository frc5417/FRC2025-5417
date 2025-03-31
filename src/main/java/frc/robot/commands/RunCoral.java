// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunCoral extends Command {
  /** Creates a new RunCoral. */
  private CoralIntake m_coralIntake;
  private double power;
  private boolean terminate = false;

  public RunCoral(CoralIntake intake, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coralIntake = intake;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralIntake.setPower(this.power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralIntake.setPower(0);
    terminate = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminate;
  }
}

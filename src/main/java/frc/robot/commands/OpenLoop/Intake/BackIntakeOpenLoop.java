// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OpenLoop.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class BackIntakeOpenLoop extends Command {
  /** Creates a new IntakeOpenLoop. */
  private IntakeSubsystem m_intake;

  private double m_voltage;

  public BackIntakeOpenLoop(IntakeSubsystem intake, double voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_voltage = voltage;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setBackIntakeVoltage(m_voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setBackIntakeVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

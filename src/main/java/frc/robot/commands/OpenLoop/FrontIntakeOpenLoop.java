// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OpenLoop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class FrontIntakeOpenLoop extends Command {

  private IntakeSubsystem m_Intake;
  private double m_voltage;
  /** Creates a new FrontIntakeOpenLoop. */
  public FrontIntakeOpenLoop(IntakeSubsystem m_Intake, double m_voltage) {
    this.m_Intake = m_Intake;
    this.m_voltage = m_voltage;
    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.setFrontIntakeVoltage(m_voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.setFrontIntakeVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

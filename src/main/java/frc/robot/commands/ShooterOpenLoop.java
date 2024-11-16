// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterOpenLoop extends Command {
  /** Creates a new ShooterOpenLoop. */
  private ShooterSubsystem m_shooter;

  private double m_voltage;

  public ShooterOpenLoop(ShooterSubsystem shooter, double voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_voltage = voltage;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setVoltage(m_voltage, m_voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setVoltage(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

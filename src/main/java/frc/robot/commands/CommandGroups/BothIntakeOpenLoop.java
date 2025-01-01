// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class BothIntakeOpenLoop extends Command {
  private IntakeSubsystem mIntake;
  private double frontVoltage;
  private double backVoltage;
  /** Creates a new BothIntakeOpenLoop. */
  public BothIntakeOpenLoop(IntakeSubsystem mIntake, double frontVoltage, double backVoltage) {
    this.mIntake = mIntake;
    this.frontVoltage = frontVoltage;
    this.backVoltage = backVoltage;
    addRequirements(mIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.setVoltage(frontVoltage, backVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.setVoltage(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

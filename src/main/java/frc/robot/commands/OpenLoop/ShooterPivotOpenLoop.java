// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OpenLoop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

public class ShooterPivotOpenLoop extends Command {
  private ShooterPivotSubsystem mPivot;
  private double voltage;
  /** Creates a new ShooterPivotOpenLoop. */
  public ShooterPivotOpenLoop(ShooterPivotSubsystem mPivot, double voltage) {
    this.mPivot = mPivot;
    this.voltage = voltage;
    addRequirements(mPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mPivot.setVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mPivot.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

public class SetPivotAngle extends Command {
  private final ShooterPivotSubsystem shooterPivot;
  private final double targetAngle;
  private final boolean shouldHold;

  /** Creates a new SetPivotAngle. */
  public SetPivotAngle(ShooterPivotSubsystem shooterPivot, double targetAngle, boolean shouldHold) {
    this.shooterPivot = shooterPivot;
    this.targetAngle = targetAngle;
    this.shouldHold = shouldHold;
    addRequirements(shooterPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(shooterPivot.getShooterPivotAngle() - targetAngle) <= 5) {
      shooterPivot.setPositionVoltageAngle(targetAngle);
    } else {
      shooterPivot.setMotionMagicAngle(targetAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!shouldHold) {
      shooterPivot.setVoltage(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(shooterPivot.getShooterPivotAngle() - targetAngle) <= 1.0;
  }
}

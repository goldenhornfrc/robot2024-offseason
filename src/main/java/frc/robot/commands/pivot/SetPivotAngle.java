// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

public class SetPivotAngle extends Command {
  private final ShooterPivotSubsystem shooterPivot;
  private final double targetAngle;
  private final boolean shouldHold;
  private boolean slot1 = false;

  /** Creates a new SetPivotAngle. */
  public SetPivotAngle(ShooterPivotSubsystem shooterPivot, double targetAngle, boolean shouldHold) {
    this.shooterPivot = shooterPivot;
    this.targetAngle = targetAngle;
    this.shouldHold = shouldHold;
    this.slot1 = false;
    addRequirements(shooterPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SetPivotAngle(
      ShooterPivotSubsystem shooterPivot, double targetAngle, boolean shouldHold, boolean slot1) {
    this.shooterPivot = shooterPivot;
    this.targetAngle = targetAngle;
    this.shouldHold = shouldHold;
    this.slot1 = slot1;
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
      if (!slot1) {
        shooterPivot.setPositionVoltageAngle(targetAngle);
      } else {
        shooterPivot.setPositionVoltageAngleSlot1(targetAngle);
      }
    } else {
      if (!slot1) {

        shooterPivot.setMotionMagicAngle(targetAngle);
      } else {
        shooterPivot.setMotionMagicAngleSlot1(targetAngle);
      }
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
    return Math.abs(shooterPivot.getShooterPivotAngle() - targetAngle) <= 1.5;
  }
}

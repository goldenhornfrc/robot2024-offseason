// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.InterpolatingDouble;
import frc.robot.Constants;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class SetPivotAngleDist extends Command {
  private final ShooterPivotSubsystem shooterPivot;
  private final VisionSubsystem vision;
  private final boolean shouldHold;

  /** Creates a new SetPivotAngle. */
  public SetPivotAngleDist(
      ShooterPivotSubsystem shooterPivot, VisionSubsystem vision, boolean shouldHold) {
    this.shooterPivot = shooterPivot;
    this.vision = vision;
    this.shouldHold = shouldHold;

    addRequirements(shooterPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SetPivotAngleDist(ShooterPivotSubsystem shooterPivot, VisionSubsystem vision) {
    this.shooterPivot = shooterPivot;
    this.vision = vision;
    this.shouldHold = true;

    addRequirements(shooterPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distance = vision.getDistanceToTarget();

    double targetAngle =
        Constants.kPivotMap.getInterpolated(new InterpolatingDouble(Double.valueOf(distance)))
            .value;
    // System.out.println(targetAngle);

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
    return false;
  }
}

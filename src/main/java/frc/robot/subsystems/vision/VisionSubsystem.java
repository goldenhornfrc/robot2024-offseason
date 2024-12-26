// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private final VisionIO io;

  private final VisionIOInputs inputs = new VisionIOInputs();

  private VisionResult targetResult;
  private VisionResult objectResult;
  private double distanceToTarget;
  private static final double limelightMountPitch = -25.0;
  private static final double limelightMountHeight = 0.2913;

  public VisionSubsystem(VisionIO io) {
    this.io = io;
    io.setTagLimelightLED(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    targetResult = inputs.targetResult;
    objectResult = inputs.objectResult;

    VisionResult targetInfo = getTargetInfo();
    if (targetInfo.targetValid) {
      distanceToTarget = inputs.targetDistance;

      SmartDashboard.putNumber("Distance to speaker", distanceToTarget);

      SmartDashboard.putNumber("TY from LL", targetInfo.targetTy);
    }
  }

  public VisionResult getTargetInfo() {
    return targetResult;
  }

  public VisionResult getObjectInfo() {
    return objectResult;
  }

  public double getDistanceToTarget() {
    return distanceToTarget;
  }

  public double getDistanceToNote(double targetY) {
    double goal_theta = Math.toRadians(limelightMountPitch) + Math.toRadians(targetY);
    double height_diff = 0.025 - limelightMountHeight;

    return height_diff / Math.tan(goal_theta);
  }

  public Command blinkTagLimelight() {
    return new InstantCommand(() -> io.blinkTagLimelight())
        .andThen(
            new WaitCommand(1)
                .andThen(
                    new InstantCommand(
                        () -> {
                          io.setTagLimelightLED(true);
                          RobotContainer.intake.setShouldBlink(false);
                        })));
  }
}

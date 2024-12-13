// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private final VisionIO io;

  private final VisionIOInputs inputs = new VisionIOInputs();

  private VisionResult targetResult;
  private VisionResult objectResult;
  private double distanceToTarget;

  public VisionSubsystem(VisionIO io) {
    this.io = io;
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

  public Command blinkIntakeLimelight() {
    return new InstantCommand(() -> io.blinkIntakeLimelight())
        .andThen(
            new WaitCommand(1).andThen(new InstantCommand(() -> io.stopBlinkingIntakeLimelight())));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private final VisionIO io;

  private final VisionIOInputs inputs = new VisionIOInputs();

  private VisionResult targetResult;
  private VisionResult objectResult;

  public VisionSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var in = io.updateInputs(inputs);
    targetResult = in.targetResult;
    objectResult = in.objectResult;

    VisionResult targetInfo = getTargetInfo();
    if (targetInfo.targetValid) {
      SmartDashboard.putNumber(
          "Distance to speaker",
          Constants.kDistanceMap.getInterpolated(new InterpolatingDouble(targetInfo.targetTy))
              .value);

      SmartDashboard.putNumber("TY from LL", targetInfo.targetTy);
    }
  }

  public VisionResult getTargetInfo() {
    return targetResult;
  }

  public VisionResult getObjectInfo() {
    return objectResult;
  }

  public static class VisionResult {
    public double targetTx = 0;
    public double targetTy = 0;
    public boolean targetValid = false;

    public VisionResult(double tx, double ty, boolean valid) {
      targetTx = tx;
      targetTy = ty;
      targetValid = valid;
    }
  }
}

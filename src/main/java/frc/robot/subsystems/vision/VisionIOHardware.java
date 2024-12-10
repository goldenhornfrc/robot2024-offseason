// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.team254.lib.pathplanner.LimelightHelpers;
import frc.robot.subsystems.vision.VisionSubsystem.VisionResult;

/** Add your docs here. */
public class VisionIOHardware implements VisionIO {

  private final String objectLLName = "limelight-two";
  private final String tagLLName = "limelight";

  @Override
  public VisionIOInputs updateInputs(VisionIOInputs inputs) {
    double objectTx = LimelightHelpers.getTX(objectLLName);
    double objectTy = LimelightHelpers.getTY(objectLLName);
    boolean objectValid = LimelightHelpers.getTV(objectLLName);
    inputs.objectResult = new VisionResult(objectTx, objectTy, objectValid);

    double targetTx = LimelightHelpers.getTX(tagLLName);
    double targetTy = LimelightHelpers.getTY(tagLLName);
    boolean targetValid = LimelightHelpers.getTV(tagLLName);
    inputs.targetResult = new VisionResult(targetTx, targetTy, targetValid);

    return inputs;
  }
}

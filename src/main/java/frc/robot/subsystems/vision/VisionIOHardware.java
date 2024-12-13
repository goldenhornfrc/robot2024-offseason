// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.team254.lib.pathplanner.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.lib.InterpolatingDouble;

/** Add your docs here. */
public class VisionIOHardware implements VisionIO {

  private final String objectLLName = "limelight-two";
  private final String tagLLName = "limelight";

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double objectTx = LimelightHelpers.getTX(objectLLName);
    double objectTy = LimelightHelpers.getTY(objectLLName);
    boolean objectValid = LimelightHelpers.getTV(objectLLName);
    inputs.objectResult = new VisionResult(objectTx, objectTy, objectValid);

    double targetTx = LimelightHelpers.getTX(tagLLName);
    double targetTy = LimelightHelpers.getTY(tagLLName);
    boolean targetValid = LimelightHelpers.getTV(tagLLName);
    inputs.targetResult = new VisionResult(targetTx, targetTy, targetValid);
    inputs.targetDistance =
        Constants.kDistanceMap.getInterpolated(new InterpolatingDouble(Double.valueOf(targetTy)))
            .value;
  }

  @Override
  public void blinkTagLimelight() {
    LimelightHelpers.setLEDMode_ForceBlink(tagLLName);
  }

  @Override
  public void setTagLimelightLED(boolean state) {
    if (state) {
      LimelightHelpers.setLEDMode_ForceOn(tagLLName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(tagLLName);
    }
  }
}

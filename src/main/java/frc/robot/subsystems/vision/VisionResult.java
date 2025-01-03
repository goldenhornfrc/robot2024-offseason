// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

/** Add your docs here. */
public class VisionResult {
  public double targetTx = 0;
  public double targetTy = 0;
  public boolean targetValid = false;

  public VisionResult(double tx, double ty, boolean valid) {
    targetTx = tx;
    targetTy = ty;
    targetValid = valid;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

/** Add your docs here. */
public interface VisionIO {

  public static class VisionIOInputs {
    public VisionResult targetResult = new VisionResult(0, 0, false);
    public VisionResult objectResult = new VisionResult(0, 0, false);
    public double targetDistance = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void blinkTagLimelight() {}

  public default void setTagLimelightLED(boolean state) {}
}

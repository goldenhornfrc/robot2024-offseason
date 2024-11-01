// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    double leftRPM = 0.0;
    double rightRPM = 0.0;

    double leftVoltage = 0.0;
    double rightVoltage = 0.0;
    
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltage(double leftVoltage,double rightVoltage){}
}
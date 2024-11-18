// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void setVoltage(double leftVoltage, double rightVoltage) {
    io.setVoltage(leftVoltage, rightVoltage);
  }

  public void setTargetRPM(double leftRPM, double rightRPM) {
    io.setTargetRPM(leftRPM, rightRPM);
  }

  public double getLeftMotorRPM() {
    return io.getLeftMotorRPM();
  }

  public double getRightMotorRPM() {
    return io.getRightMotorRPM();
  }

  public double getLeftTargetRPM() {
    return io.getLeftTargetRPM();
  }

  public double getRightTargetRPM() {
    return io.getRightTargetRPM();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LeftRPM", getLeftMotorRPM());
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", (LoggableInputs) inputs);
    // This method will be called once per scheduler run
  }
}

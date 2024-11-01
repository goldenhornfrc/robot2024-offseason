// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;

public class FeederSubsystem extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputs inputs = new FeederIOInputs();

  private final DigitalInput digitalInput = new DigitalInput(0);

  /** Creates a new feederSubsystem. */
  public enum feederState {
    IDLE,
    SPINNING
  }

  public FeederSubsystem(FeederIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public boolean getFeederSensorValue() {
    return digitalInput.get();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", (LoggableInputs) inputs);  //!!!!!

    // This method will be called once per scheduler run
  }
}
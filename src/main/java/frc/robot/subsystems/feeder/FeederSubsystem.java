// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FeederSubsystem extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final DigitalInput frontSensor = new DigitalInput(0);
  private final DigitalInput backSensor = new DigitalInput(1);

  public FeederSubsystem(FeederIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    // io.setVoltage(voltage);
    io.setVoltage(voltage);
  }

  public boolean getBackSensor() {
    return !backSensor.get();
  }

  public boolean getFrontSensor() {
    return !frontSensor.get();
  }

  public boolean getBothSensor() {
    if (!frontSensor.get() && !backSensor.get()) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", (LoggableInputs) inputs); // !!!!!

    // This method will be called once per scheduler run
  }
}

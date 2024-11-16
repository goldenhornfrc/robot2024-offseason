// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  /** Creates a new IntakeSubsystem. */
  public enum IntakeState {
    IDLE,
    INTAKE,
    OUTTAKE
  }

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public void setVoltage(double frontVoltage,double backVoltage) {
    io.setVoltage(frontVoltage,backVoltage);
  }

  public void setBackIntakeVoltage(double backVoltage){
    io.setBackIntakeVoltage(backVoltage);
  }


  public void setFrontIntakeVoltage(double frontVoltage){
    io.setBackIntakeVoltage(frontVoltage);
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    // This method will be called once per scheduler run
  }
}
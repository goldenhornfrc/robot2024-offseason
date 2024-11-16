// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.TalonFXUtil;

/** Add your docs here. */
public class FeederIOFalcon implements FeederIO {
  // motor
  private final TalonFX feederMotor = new TalonFX(5);
  // StatusSignals
  private StatusSignal<Double> feederMotorVolts = feederMotor.getMotorVoltage();

  public FeederIOFalcon() {
    configIntakeTalonFX(feederMotor);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, feederMotorVolts);
  }

  public void configIntakeTalonFX(TalonFX talon) {
    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; //TODO
    config.Feedback.FeedbackRotorOffset = 0.0;  //TODO
    config.Feedback.RotorToSensorRatio = 1.0;   //TODO
    config.Feedback.SensorToMechanismRatio = 1.0; //TODO

    TalonFXUtil.applyAndCheckConfiguration(talon, config);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(feederMotorVolts);

    feederMotorVolts = feederMotor.getMotorVoltage();
  }

  @Override
  public void setVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class ShooterIOKraken implements ShooterIO {
  private final TalonFX leftMotor = new TalonFX(ShooterConstants.leftMotorID); // left
  private final TalonFX rightMotor = new TalonFX(1); // right

  private final StatusSignal<Double> leftRPM = leftMotor.getVelocity();
  private final StatusSignal<Double> rightRPM = rightMotor.getVelocity();
  
  public ShooterIOKraken() {
    configShooterTalonFX(leftMotor);
    configShooterTalonFX(rightMotor);
    leftMotor.setInverted(false);
    rightMotor.setInverted(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftRPM, rightRPM);
  }

  public void configShooterTalonFX(TalonFX talon) {
    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = false;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.Audio.BeepOnBoot = true;
    config.Audio.BeepOnConfig = false;

    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot1.kS = 0;
    config.Slot1.kV = 0;
    config.Slot1.kA = 0;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(leftRPM, rightRPM);

    inputs.leftRPM = leftMotor.getVelocity().getValueAsDouble() * 60;
    inputs.rightRPM = leftMotor.getVelocity().getValueAsDouble() * 60;
  }

  @Override 
  public void setVoltage(double leftVoltage,double rightVoltage){
    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);
  }

  
}
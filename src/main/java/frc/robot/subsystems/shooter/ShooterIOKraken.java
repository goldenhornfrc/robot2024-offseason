// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.util.TalonFXUtil;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class ShooterIOKraken implements ShooterIO {
  private final TalonFX leftMotor = new TalonFX(ShooterConstants.leftMotorID, "Canivore");
  private final TalonFX rightMotor = new TalonFX(ShooterConstants.rightMotorID, "Canivore");

  private final StatusSignal<Double> leftRPM = leftMotor.getVelocity();
  private final StatusSignal<Double> rightRPM = rightMotor.getVelocity();
  private final StatusSignal<Double> leftVoltage = leftMotor.getMotorVoltage();
  private final StatusSignal<Double> rightVoltage = rightMotor.getMotorVoltage();

  private double leftTargetRPM, rightTargetRPM = 0.0;

  public ShooterIOKraken() {
    configShooterTalonFX(leftMotor);
    configShooterTalonFX(rightMotor);
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftRPM, rightRPM, leftVoltage, rightVoltage);
  }

  public void configShooterTalonFX(TalonFX talon) {
    talon.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.DutyCycleNeutralDeadband = 0.02;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = 21.0 / 15.0;

    config.Audio.BeepOnBoot = true;
    config.Audio.BeepOnConfig = true;

    config.Slot0.kP = 0.18;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    config.Slot0.kS = 0;
    config.Slot0.kV = 0.17;
    config.Slot0.kA = 0;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.00;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    config.CurrentLimits.SupplyCurrentLimit = 55.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    TalonFXUtil.applyAndCheckConfiguration(talon, config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(leftRPM, rightRPM, leftVoltage, rightVoltage);

    inputs.leftRPM = leftMotor.getVelocity().getValueAsDouble() * 60;
    inputs.rightRPM = leftMotor.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void setVoltage(double leftVoltage, double rightVoltage) {
    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);
  }

  @Override
  public void setTargetRPM(double leftRPM, double rightRPM) {
    leftTargetRPM = leftRPM;
    rightTargetRPM = rightRPM;

    leftMotor.setControl(new VelocityVoltage(leftRPM / 60.0).withSlot(0));
    rightMotor.setControl(new VelocityVoltage(rightRPM / 60.0).withSlot(0));
  }

  @Override
  public double getLeftMotorRPM() {
    return leftMotor.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public double getRightMotorRPM() {
    return rightMotor.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public double getLeftTargetRPM() {
    return leftTargetRPM;
  }

  @Override
  public double getRightTargetRPM() {
    return rightTargetRPM;
  }
}

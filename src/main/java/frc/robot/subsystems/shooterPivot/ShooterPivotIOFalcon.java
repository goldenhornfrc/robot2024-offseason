// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterPivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.TalonFXUtil;

/** Add your docs here. */
public class ShooterPivotIOFalcon implements ShooterPivotIO {
  private final TalonFX shooterPivotMotor = new TalonFX(21, "Canivore");

  private final PIDController pid = new PIDController(0.1, 0, 0);
  // Status signals
  private final StatusSignal<Double> shooterPosition = shooterPivotMotor.getPosition();
  private final StatusSignal<Double> pivotVolts = shooterPivotMotor.getMotorVoltage();
  //

  private double targetAngle = 0.0;

  public ShooterPivotIOFalcon() {
    configureShooterPivotTalon();
    shooterPivotMotor.setInverted(true);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, shooterPosition, pivotVolts);
  }

  public void configureShooterPivotTalon() {

    shooterPivotMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.DutyCycleNeutralDeadband = 0.02;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = false;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.FeedbackRotorOffset = 0.0;
    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = 144.0;

    config.Audio.BeepOnBoot = true;
    config.Audio.BeepOnConfig = true;

    config.Slot0.kS = 0;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;

    config.Slot0.kP = 144 * 4;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.0;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.05;

    config.MotionMagic.MotionMagicCruiseVelocity = 1;
    config.MotionMagic.MotionMagicAcceleration = 1.2;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 119.0 / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.5 / 360.0;

    TalonFXUtil.applyAndCheckConfiguration(shooterPivotMotor, config);
  }

  @Override
  public void updateInputs(ShooterPivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(shooterPosition, pivotVolts);
    inputs.pivotVolts = shooterPivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.shooterPosition = shooterPivotMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    shooterPivotMotor.setVoltage(voltage);
  }

  @Override
  public void setMotionMagicAngle(double angle) {
    targetAngle = angle;
    shooterPivotMotor.setControl(new MotionMagicVoltage(angle / 360.0).withSlot(0));
  }

  @Override
  public void setPositionVoltageAngle(double angle) {
    targetAngle = angle;
    shooterPivotMotor.setControl(new PositionVoltage(angle / 360).withSlot(0));
  }

  @Override
  public double getShooterPivotAngle() {
    return shooterPivotMotor.getPosition().getValueAsDouble() * 360;
  }

  @Override
  public double getTargetAngle() {
    return targetAngle;
  }

  /** Creates a new ShooterPivot. */
}

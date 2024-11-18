// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.util.SparkMaxUtil;

/** Add your docs here. */
public class IntakeIOSpark implements IntakeIO{
    private final CANSparkMax frontIntake = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax backIntake = new CANSparkMax(3, MotorType.kBrushless);

    public IntakeIOSpark() {
       configBackIntakeSpark();
       configFrontIntakeSpark();
    }

 private void configFrontIntakeSpark() {
        SparkMaxUtil.restoreFactoryDefaults(frontIntake);

        
        frontIntake.setInverted(false);


        final int motorID = frontIntake.getDeviceId();

        SparkMaxUtil.applyParameter(
                () -> frontIntake.setIdleMode(IdleMode.kCoast),
                () -> true,
                "Failed to set Idle Mode! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () ->
                        frontIntake
                                .getEncoder()
                                .setPositionConversionFactor(0),//TODO
                () -> true,
                "Failed to set Position Conversion Factor! - ID: " + motorID);

        SparkMaxUtil.applyParameter(
                () ->
                        frontIntake
                                .getEncoder()
                                .setVelocityConversionFactor(0),//TODO
                () -> true,
                "Failed to set Velocity Conversion Factor! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () -> frontIntake.setOpenLoopRampRate(0.05),
                () -> true,
                "Failed to set Open Loop Ramp! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () -> frontIntake.setClosedLoopRampRate(0),
                () -> true,
                "Failed to set Closed Loop Ramp! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () -> frontIntake.setSmartCurrentLimit(40),
                () -> true,
                "Failed to set Smart Current Limit! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () -> frontIntake.enableVoltageCompensation(12.0),
                () -> true,
                "Failed to enable Voltage Compensation! - ID: " + motorID);

        SparkMaxUtil.burnFlash(frontIntake);
    }
     private void configBackIntakeSpark() {
        SparkMaxUtil.restoreFactoryDefaults(backIntake);

        
        backIntake.setInverted(false);


        final int motorID = backIntake.getDeviceId();

        SparkMaxUtil.applyParameter(
                () -> backIntake.setIdleMode(IdleMode.kCoast),
                () -> true,
                "Failed to set Idle Mode! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () ->
                        backIntake
                                .getEncoder()
                                .setPositionConversionFactor(0),//TODO
                () -> true,
                "Failed to set Position Conversion Factor! - ID: " + motorID);

        SparkMaxUtil.applyParameter(
                () ->
                        backIntake
                                .getEncoder()
                                .setVelocityConversionFactor(0),//TODO
                () -> true,
                "Failed to set Velocity Conversion Factor! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () -> backIntake.setOpenLoopRampRate(0.05),
                () -> true,
                "Failed to set Open Loop Ramp! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () -> backIntake.setClosedLoopRampRate(0),
                () -> true,
                "Failed to set Closed Loop Ramp! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () -> backIntake.setSmartCurrentLimit(40),
                () -> true,
                "Failed to set Smart Current Limit! - ID: " + motorID);
        SparkMaxUtil.applyParameter(
                () -> backIntake.enableVoltageCompensation(12.0),
                () -> true,
                "Failed to enable Voltage Compensation! - ID: " + motorID);

        SparkMaxUtil.burnFlash(backIntake);
    }



    @Override
    public  void setVoltage(double frontVoltage, double backVoltage) {
        frontIntake.setVoltage(frontVoltage);
        backIntake.setVoltage(backVoltage);
    }

    @Override
    public void setFrontIntakeVoltage(double frontVoltage){
        frontIntake.setVoltage(frontVoltage);
    }
    @Override
    public void setBackIntakeVoltage(double backVoltage){
        backIntake.setVoltage(backVoltage);
    }   
}
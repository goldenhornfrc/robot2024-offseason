// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterLeft = new TalonFX(45, "Canivore");
  private final TalonFX shooterRight = new TalonFX(3, "Canivore");

  public void setVoltage(double leftVoltage, double rightVoltage) {
    io.setVoltage(leftVoltage, rightVoltage);
  }

  public void setTargetRPM(double leftRPM,double rightRPM){
    io.setTargetRPM(leftRPM, rightRPM);
  }

  public double getLeftMotorRPM(){
    io.getRightMotorRPM();
  }

  public double getRightMotorRPM(){
    io.getRightMotorRPM();
  }

  public double getLeftTargetRPM(){
    io.getLeftTargetRPM();
  }

  public double getRightTargetRPM(){
    io.getRightTargetRPM();
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("Shooter", (LoggableInputs) inputs);
    // This method will be called once per scheduler run
  }
}

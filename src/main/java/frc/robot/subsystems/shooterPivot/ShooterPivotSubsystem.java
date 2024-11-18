// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterPivotSubsystem extends SubsystemBase {
  private final ShooterPivotIO io;
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  private final Translation3d pivotPosition = new Translation3d(0, 0.05, 0.63);

  StructPublisher<Pose3d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();

  public ShooterPivotSubsystem(ShooterPivotIO io) {
    this.io = io;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setMotionMagicAngle(double targetAngle) {
    io.setMotionMagicAngle(targetAngle);
  }

  public void setPositionVoltageAngle(double angle) {
    io.setPositionVoltageAngle(angle);
  }

  public double getShooterPivotAngle() {
    return io.getShooterPivotAngle();
  }

  public double getTargetAngle() {
    return io.getTargetAngle();
  }

  public double getPosition() {
    return inputs.shooterPosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Pivot", inputs);
    SmartDashboard.putNumber("Pivot Angle", io.getShooterPivotAngle());
    publisher.set(new Pose3d(pivotPosition, new Rotation3d(io.getShooterPivotAngle(), 0, 0)));
    // This method will be called once per scheduler run
  }
}

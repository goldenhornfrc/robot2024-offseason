// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.object;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.swerve.SwerveRequest;

public class ObjectDetection extends Command {

  private DriveSubsystem mDrive;
  private VisionSubsystem mVision;
  private SwerveRequest.RobotCentric heading =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(frc.robot.util.swerve.SwerveModule.DriveRequestType.Velocity);

  private PIDController driveController = new PIDController(0.05, 0, 0.002);
  private PIDController thetaController = new PIDController(0.1, 0, 0.0);
  /** Creates a new ObejctDetection. */
  public ObjectDetection(DriveSubsystem mDrive, VisionSubsystem mVision) {
    this.mDrive = mDrive;
    this.mVision = mVision;
    addRequirements(mDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.setTolerance(0.2);
    driveController.setTolerance(0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var info = mVision.getObjectInfo();
    var xVel = 0.0;
    var rotVel = 0.0;

    if (info.targetValid) {

      if (info.targetTy <= 0.0) {
        thetaController.setPID(0.03, 0, 0);
        thetaController.setTolerance(1.0);
      } else {
        thetaController.setPID(0.1, 0, 0);
        thetaController.setTolerance(0.2);
        driveController.setTolerance(0.8);
      }
      xVel = driveController.calculate(info.targetTy, -16.0);
      rotVel = thetaController.calculate(info.targetTx, 0.0);
    } else {
      xVel = 0;
      rotVel = 0;
    }

    SmartDashboard.putNumber("X Velocity", xVel);
    SmartDashboard.putNumber("Rot Velocity", rotVel);
    mDrive.setControl(
        heading
            .withVelocityX(xVel)
            .withVelocityY(0)
            .withRotationalRate(rotVel)
            .withRotationalDeadband(0.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (driveController.atSetpoint() && thetaController.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}

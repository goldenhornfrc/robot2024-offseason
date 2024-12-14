// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.object;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.swerve.SwerveRequest;

public class ObjectDetection extends Command {

  private DriveSubsystem mDrive;
  private VisionSubsystem mVision;
  private SwerveRequest.RobotCentric heading =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(frc.robot.util.swerve.SwerveModule.DriveRequestType.Velocity);

  private PIDController driveController = new PIDController(6.0, 0, 0.0);
  private PIDController thetaController = new PIDController(0.1, 0, 0.0);
  private final double limelightMountYaw = 33.0;
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
    driveController.setTolerance(0.02);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var info = mVision.getObjectInfo();
    double distance = mVision.getDistanceToNote(info.targetTy);
    var xVel = 0.0;
    var yVel = 0.0;
    var rotVel = 0.0;

    if (info.targetValid) {
      SmartDashboard.putNumber("Dist to note", distance);
      double noteXDiff =
          Math.sin(Math.toRadians(90.0 - (limelightMountYaw - info.targetTx))) * distance;
      double noteYDiff =
          (Math.cos(Math.toRadians(90.0 - (limelightMountYaw - info.targetTx))) * distance) - 0.255;

      SmartDashboard.putNumber("Note X Diff", noteXDiff);
      SmartDashboard.putNumber("Note Y Diff", noteYDiff);

      xVel = driveController.calculate(noteXDiff, 0.2);
      yVel = driveController.calculate(noteYDiff, 0.0);

      xVel = MathUtil.clamp(xVel, -1.5, 1.5);
      yVel = MathUtil.clamp(yVel, -1.5, 1.5);
    } else {
      xVel = 0;
      yVel = 0;
      rotVel = 0;
    }

    SmartDashboard.putNumber("X Velocity", xVel);
    SmartDashboard.putNumber("Rot Velocity", rotVel);
    mDrive.setControl(
        heading
            .withVelocityX(xVel)
            .withVelocityY(yVel)
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
    if ((driveController.atSetpoint() && thetaController.atSetpoint())
        || RobotContainer.feeder.getFrontSensor()) {
      return true;
    } else {
      return false;
    }
  }
}

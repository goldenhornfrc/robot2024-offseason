// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveState;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.swerve.SwerveModule;
import frc.robot.util.swerve.SwerveRequest;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommand extends Command {
  private DriveSubsystem mDrivetrain;
  private VisionSubsystem mVision;
  private DoubleSupplier mThrottleSupplier, mStrafeSupplier, mTurnSupplier;
  private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
  private Rotation2d targetHeading;

  private SwerveRequest.FieldCentric driveNoHeading =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.DriveConstants.kMaxVel * 0.05) // Add a 5% deadband in open loop
          .withRotationalDeadband(
              Constants.DriveConstants.kMaxAngularVel
                  * Constants.DriveConstants.kSteerJoystickDeadband)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  private SwerveRequest.FieldCentricFacingAngle driveWithHeading =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(Constants.DriveConstants.kMaxVel * 0.05)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  private SwerveRequest.FieldCentricFacingAngle visionAlignRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(Constants.DriveConstants.kMaxVel * 0.05)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  /** Creates a new DriveCommand. */
  public DriveCommand(
      DriveSubsystem drivetrain,
      VisionSubsystem vision,
      DoubleSupplier throttle,
      DoubleSupplier strafe,
      DoubleSupplier turn) {
    mDrivetrain = drivetrain;
    mThrottleSupplier = throttle;
    mStrafeSupplier = strafe;
    mTurnSupplier = turn;
    mVision = vision;

    driveWithHeading.HeadingController.setPID(
        Constants.DriveConstants.kHeadingControllerP,
        Constants.DriveConstants.kHeadingControllerI,
        Constants.DriveConstants.kHeadingControllerD);
    driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    visionAlignRequest.HeadingController = mDrivetrain.getVisionController();
    visionAlignRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mHeadingSetpoint = Optional.empty();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveState currentState = mDrivetrain.getDriveState();
    double throttle = mThrottleSupplier.getAsDouble() * Constants.DriveConstants.kMaxVel;
    double strafe = mStrafeSupplier.getAsDouble() * Constants.DriveConstants.kMaxVel;
    double turnFieldFrame = mTurnSupplier.getAsDouble();
    double throttleFieldFrame = Robot.getAlliance() == Alliance.Red ? throttle : -throttle;
    double strafeFieldFrame = Robot.getAlliance() == Alliance.Red ? strafe : -strafe;

    if (currentState == DriveState.OPEN_LOOP) {
      if (Math.abs(turnFieldFrame) >= 0.1) {
        turnFieldFrame = turnFieldFrame * Constants.DriveConstants.kMaxAngularVel;
        mDrivetrain.setControl(
            driveNoHeading
                .withVelocityX(throttleFieldFrame)
                .withVelocityY(strafeFieldFrame)
                .withRotationalRate(turnFieldFrame));
        mHeadingSetpoint = Optional.empty();
        Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
      } else {
        if (mHeadingSetpoint.isEmpty()) {
          mHeadingSetpoint = Optional.of(mDrivetrain.getPose().getRotation());
        }
        mDrivetrain.setControl(
            driveWithHeading
                .withVelocityX(throttleFieldFrame)
                .withVelocityY(strafeFieldFrame)
                .withTargetDirection(mHeadingSetpoint.get()));
        Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
        Logger.recordOutput(
            "DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
      }
    }
    if (currentState == DriveState.HEADING_LOCK) {
      mHeadingSetpoint = Optional.empty();
      targetHeading = Rotation2d.fromDegrees(mDrivetrain.getTargetHeading());

      mDrivetrain.setControl(
          driveWithHeading
              .withVelocityX(throttleFieldFrame)
              .withVelocityY(strafeFieldFrame)
              .withTargetDirection(targetHeading));

      Logger.recordOutput("DriveCommand/Mode", "TargetHeading");
      Logger.recordOutput("DriveCommand/targetHeading", targetHeading.getDegrees());
    }

    if (currentState == DriveState.INTAKE_STATE) {

      var xVel = MathUtil.clamp(throttleFieldFrame, -1.0, 1.0);
      var yVel = MathUtil.clamp(strafeFieldFrame, -1.0, 1.0);

      if (Math.abs(turnFieldFrame) >= 0.1) {
        turnFieldFrame = turnFieldFrame * Constants.DriveConstants.kMaxAngularVel;
        mDrivetrain.setControl(
            driveNoHeading
                .withVelocityX(xVel)
                .withVelocityY(yVel)
                .withRotationalRate(turnFieldFrame));
        mHeadingSetpoint = Optional.empty();
        Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
      } else {
        if (mHeadingSetpoint.isEmpty()) {
          mHeadingSetpoint = Optional.of(mDrivetrain.getPose().getRotation());
        }
        mDrivetrain.setControl(
            driveWithHeading
                .withVelocityX(xVel)
                .withVelocityY(yVel)
                .withTargetDirection(mHeadingSetpoint.get()));
        Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
        Logger.recordOutput(
            "DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
      }
    }

    if (currentState == DriveState.VISION_STATE) {
      var info = mVision.getTargetInfo();
      if (info.targetValid) {
        mHeadingSetpoint =
            Optional.of(
                mDrivetrain.getPose().getRotation().minus(Rotation2d.fromDegrees(info.targetTx)));
      }

      mDrivetrain.setControl(
          visionAlignRequest
              .withVelocityX(throttleFieldFrame)
              .withVelocityY(strafeFieldFrame)
              .withTargetDirection(mHeadingSetpoint.get()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mHeadingSetpoint = Optional.empty();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

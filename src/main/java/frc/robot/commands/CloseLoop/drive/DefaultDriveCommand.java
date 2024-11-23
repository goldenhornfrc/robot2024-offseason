// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveState;
import frc.robot.util.swerve.*;
import frc.robot.util.swerve.SwerveModule.DriveRequestType;
import frc.robot.util.swerve.SwerveModule.SteerRequestType;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {
  /** Creates a new DefaultDriveCommand. */
  private final DriveSubsystem m_drive;

  private final DoubleSupplier xSupplier, ySupplier, rotSupplier;
  private final BooleanSupplier fieldOrientedSupplier;

  private SwerveRequest.FieldCentricFacingAngle driveWithHeading =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(DriveConstants.kMaxVel * 0.05)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  public DefaultDriveCommand(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      BooleanSupplier fieldOriented) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
    this.fieldOrientedSupplier = fieldOriented;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DriveState currentState = m_drive.getDriveState();

    switch (currentState) {
      case OPEN_LOOP:
        // m_drive.fieldCentricHeadingLockRequest.HeadingController.setP(0.1);
        m_drive.setControl(
            fieldOrientedSupplier.getAsBoolean()
                ? new SwerveRequest.FieldCentric()
                    .withVelocityX(-xSupplier.getAsDouble() * DriveConstants.kMaxVel)
                    .withVelocityY(-ySupplier.getAsDouble() * DriveConstants.kMaxVel)
                    .withRotationalRate(-rotSupplier.getAsDouble() * DriveConstants.kMaxAngularVel)
                    .withSteerRequestType(SteerRequestType.MotionMagic)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withDeadband(0.05)
                : new SwerveRequest.RobotCentric()
                    .withVelocityX(-xSupplier.getAsDouble() * DriveConstants.kMaxVel)
                    .withVelocityY(-ySupplier.getAsDouble() * DriveConstants.kMaxVel)
                    .withRotationalRate(-rotSupplier.getAsDouble() * DriveConstants.kMaxAngularVel)
                    .withSteerRequestType(SteerRequestType.MotionMagic)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withDeadband(0.05));

        break;
      case HEADING_LOCK:
        double targetHeading = m_drive.getTargetHeading();
        m_drive.setControl(
            m_drive
                .fieldCentricHeadingLockRequest
                .withVelocityX(
                    MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.08)
                        * DriveConstants.kMaxVel
                        * (Robot.getAlliance() == Alliance.Blue ? -1 : 1))
                .withVelocityY(
                    MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.08)
                        * DriveConstants.kMaxVel
                        * (Robot.getAlliance() == Alliance.Blue ? -1 : 1))
                .withSteerRequestType(SteerRequestType.MotionMagic)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withTargetDirection(Rotation2d.fromDegrees(targetHeading)));
        break;
      default:
        m_drive.setControl(new SwerveRequest.Idle());
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

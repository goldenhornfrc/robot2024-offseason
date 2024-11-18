// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
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
    /*  m_drive.setControl(
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
            .withDeadband(0.05)); */

    m_drive.setControl(
        new SwerveRequest.RobotCentric()
            .withVelocityX(-xSupplier.getAsDouble() * DriveConstants.kMaxVel)
            .withVelocityY(-ySupplier.getAsDouble() * DriveConstants.kMaxVel)
            .withRotationalRate(-rotSupplier.getAsDouble() * DriveConstants.kMaxAngularVel)
            .withSteerRequestType(SteerRequestType.MotionMagic)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(0.05));
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

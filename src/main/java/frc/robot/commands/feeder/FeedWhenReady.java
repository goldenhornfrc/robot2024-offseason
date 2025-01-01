// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

public class FeedWhenReady extends Command {
  private final DriveSubsystem mDrive;
  private final ShooterSubsystem mShooter;
  private final FeederSubsystem mFeeder;
  private final ShooterPivotSubsystem mPivot;

  /** Creates a new FeedWhenReady. */
  public FeedWhenReady(
      DriveSubsystem mDrive,
      ShooterSubsystem mShooter,
      FeederSubsystem mFeeder,
      ShooterPivotSubsystem mPivot) {
    this.mDrive = mDrive;
    this.mShooter = mShooter;
    this.mFeeder = mFeeder;
    this.mPivot = mPivot;
    addRequirements(mFeeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Constants.kShootingParams.isShooterAtSetpoint(
            mShooter.getLeftMotorRPM(), mShooter.getLeftTargetRPM())
        && Constants.kShootingParams.isShooterAtSetpoint(
            mShooter.getRightMotorRPM(), mShooter.getRightTargetRPM())
        && Constants.kShootingParams.isShooterPivotAtSetpoint(
            mPivot.getShooterPivotAngle(), mPivot.getTargetAngle())
        && mDrive.getVisionController().atSetpoint()) {
      mFeeder.setVoltage(7);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mFeeder.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

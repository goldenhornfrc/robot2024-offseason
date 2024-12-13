// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.swerve.SwerveRequest;

public class SpeakerAlignCommand extends Command {
  private final DriveSubsystem mDrive;
  private final VisionSubsystem mVision;
  private SwerveRequest.RobotCentric heading =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(
              frc.robot.util.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

  private PIDController headingController = new PIDController(0.07, 0, 0);

  /** Creates a new SpeakerAllignCommand. */
  public SpeakerAlignCommand(DriveSubsystem mDrive, VisionSubsystem mVision) {
    this.mDrive = mDrive;
    this.mVision = mVision;
    addRequirements(mDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    headingController.setTolerance(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var info = mVision.getTargetInfo();
    double rotVel = 0;
    if (info.targetValid) {
      rotVel = headingController.calculate(info.targetTx, 0);
    }
    mDrive.setControl(
        heading
            .withRotationalRate(rotVel)
            .withRotationalDeadband(0.051)
            .withVelocityX(0)
            .withVelocityY(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (headingController.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}

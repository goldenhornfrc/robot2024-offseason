// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.object;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.swerve.SwerveRequest;

public class ObejctDetection extends Command {

  private DriveSubsystem mDrive;
  private SwerveRequest.RobotCentric heading =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(frc.robot.util.swerve.SwerveModule.DriveRequestType.Velocity);
  private PIDController txController = new PIDController(0, 0, 0);
  private PIDController thetaController = new PIDController(0, 0, 0);
  /** Creates a new ObejctDetection. */
  public ObejctDetection(DriveSubsystem mDrive) {
    this.mDrive = mDrive;
    addRequirements(mDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.setTolerance(0);
    txController.setTolerance(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var txHeading = txController.calculate(0, 0);
    var thetaHeading = thetaController.calculate(0, 0);

    mDrive.setControl(
        heading.withVelocityX(txHeading).withVelocityY(0).withRotationalRate(thetaHeading));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

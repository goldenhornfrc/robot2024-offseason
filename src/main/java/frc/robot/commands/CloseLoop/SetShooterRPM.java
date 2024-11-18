// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SetShooterRPM extends Command {
  private ShooterSubsystem shooter;
  private double leftRPM;
  private double rightRPM;
  /** Creates a new setShooterRPM. */
  public SetShooterRPM(ShooterSubsystem shooter, double leftRPM, double rightRPM) {
    this.shooter = shooter;
    this.leftRPM = leftRPM;
    this.rightRPM = rightRPM;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setTargetRPM(leftRPM, rightRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

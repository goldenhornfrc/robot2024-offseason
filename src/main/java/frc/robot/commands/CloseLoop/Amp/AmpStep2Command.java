// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.Amp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CloseLoop.pivot.SetPivotAngle;
import frc.robot.commands.OpenLoop.FeederOpenLoop;
import frc.robot.commands.OpenLoop.ShooterOpenLoop;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpStep2Command extends SequentialCommandGroup {
  /** Creates a new AmpStep2Command. */
  public AmpStep2Command(
      FeederSubsystem mFeeder,
      ShooterSubsystem mShooter,
      DriveSubsystem mDrive,
      ShooterPivotSubsystem mPivot) {
    // Add your commands in the addCommands() call, e.g.
    // new ShooterOpenLoop(mShooter, 4).withTimeout(1.5).raceWith(new WaitCommand(0.7).andThen(new
    // FeederOpenLoop(mFeeder, 3)))
    /*new SetPivotAngle(mPivot, 119.3, true)
    .andThen(new ShooterOpenLoop(mShooter, 6).withTimeout(1.5))
    .raceWith(new WaitCommand(0.8).andThen(new FeederOpenLoop(mFeeder, 5))) */
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetPivotAngle(mPivot, 122, true).raceWith(new ShooterOpenLoop(mShooter, 2.8)),
        new FeederOpenLoop(mFeeder, -3).withTimeout(0.05),
        new ShooterOpenLoop(mShooter, 2.8)
            .withTimeout(1.0)
            .raceWith(new WaitCommand(0.3).andThen(new FeederOpenLoop(mFeeder, 5))));
  }
}

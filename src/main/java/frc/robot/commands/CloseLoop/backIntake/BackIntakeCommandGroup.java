// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.backIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CloseLoop.pivot.SetPivotAngle;
import frc.robot.commands.OpenLoop.BackIntakeOpenLoop;
import frc.robot.commands.OpenLoop.FeederOpenLoop;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackIntakeCommandGroup extends SequentialCommandGroup {
  /** Creates a new BackIntakeHasNote. */
  public BackIntakeCommandGroup(
      FeederSubsystem mFeeder,
      IntakeSubsystem mBackIntake,
      ShooterSubsystem mShooter,
      ShooterPivotSubsystem mPivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand();
    addCommands(
        new SetPivotAngle(mPivot, 30, true),
        Commands.race(
            Commands.waitUntil(mFeeder::getFrontSensor),
            new BackIntakeOpenLoop(mBackIntake, 6.5),
            new FeederOpenLoop(mFeeder, 3.5)));

    /*new WaitForFrontSensor(mFeeder)
            .raceWith(
                new FeederOpenLoop(mFeeder, 4), new BackIntakeOpenLoop(mBackIntake, 5)))
    .andThen(new FeederOpenLoop(mFeeder, -1.0).withTimeout(0.25)*/

  }
}

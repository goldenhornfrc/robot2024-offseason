// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.frontIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.pivot.SetPivotAngle;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FrontIntakeCommandGroup extends SequentialCommandGroup {
  /** Creates a new FrontIntakeHasNote. */
  public FrontIntakeCommandGroup(
      ShooterPivotSubsystem mPivot,
      ShooterSubsystem mShooter,
      IntakeSubsystem mFrontIntake,
      FeederSubsystem mFeeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetPivotAngle(mPivot, 0, false, true).raceWith(new ShooterOpenLoop(mShooter, -5.5)),
        new WaitCommand(0.5).raceWith(new ShooterOpenLoop(mShooter, -5.5)),
        Commands.race(
            Commands.waitUntil(mFeeder::getBackSensor),
            new FrontIntakeOpenLoop(mFrontIntake, 5.0),
            new ShooterOpenLoop(mShooter, -5.5)),
        Commands.race(
            Commands.waitUntil(mFeeder::getFrontSensor)
                .andThen(new InstantCommand(() -> mFrontIntake.setShouldBlink(true))),
            new ShooterOpenLoop(mShooter, -5.5),
            new FeederOpenLoop(mFeeder, -2.0)));
  }
}

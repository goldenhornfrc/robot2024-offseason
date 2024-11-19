// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.OpenLoop.FeederOpenLoop;
import frc.robot.commands.OpenLoop.FrontIntakeOpenLoop;
import frc.robot.commands.OpenLoop.ShooterOpenLoop;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FrontIntakeHasNote extends SequentialCommandGroup {
  /** Creates a new FrontIntakeHasNote. */
  public FrontIntakeHasNote(
      ShooterPivotSubsystem mPivot,
      ShooterSubsystem mShooter,
      IntakeSubsystem mFrontIntake,
      FeederSubsystem mFeeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetPivotAngle(mPivot, 0, false)
            .andThen(
                new WaitUntilCommand(mFeeder::getBothSensor)
                    .raceWith(
                        Commands.parallel(
                            new FrontIntakeOpenLoop(mFrontIntake, 6),
                            new ShooterOpenLoop(mShooter, -6)))
                    .andThen(
                        new WaitUntilCommand(mFeeder::getFrontSensor)
                            .raceWith(Commands.parallel(new FeederOpenLoop(mFeeder, -4))))
                    .andThen(new FeederOpenLoop(mFeeder, 2).withTimeout(0.2))));
  }
}

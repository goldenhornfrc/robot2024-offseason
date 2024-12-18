// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.backIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.CloseLoop.pivot.SetPivotAngle;
import frc.robot.commands.CloseLoop.sensors.WaitForBackSensor;
import frc.robot.commands.OpenLoop.BackIntakeOpenLoop;
import frc.robot.commands.OpenLoop.FeederOpenLoop;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackIntakeAutoCommand extends SequentialCommandGroup {
  /** Creates a new BackIntakeHasNote. */
  public BackIntakeAutoCommand(
      FeederSubsystem mFeeder,
      IntakeSubsystem mBackIntake,
      ShooterSubsystem mShooter,
      ShooterPivotSubsystem mPivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand();
    addCommands(
        new SetPivotAngle(mPivot, 50, true)
            .andThen(
                new WaitForBackSensor(mFeeder)
                    .raceWith(
                        Commands.parallel(
                            new BackIntakeOpenLoop(mBackIntake, 5),
                            new FeederOpenLoop(mFeeder, 3))))
            .andThen(
                new WaitUntilCommand(mFeeder::getFrontSensor)
                    .raceWith(
                        new FeederOpenLoop(mFeeder, 2), new BackIntakeOpenLoop(mBackIntake, 4))));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.frontIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.CloseLoop.pivot.SetPivotAngle;
import frc.robot.commands.CloseLoop.sensors.WaitForBackSensor;
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
        new SetPivotAngle(mPivot, 0, false)
            .alongWith(new ShooterOpenLoop(mShooter, 6))
            .andThen(
                new WaitForBackSensor(mFeeder)
                    .raceWith(
                        Commands.parallel(
                            new ShooterOpenLoop(mShooter, -5.5),
                            new FrontIntakeOpenLoop(mFrontIntake, 5),
                            new FeederOpenLoop(mFeeder, -1.5)))
                    .andThen(
                        new WaitUntilCommand(mFeeder::getFrontSensor)
                            .raceWith(
                                Commands.parallel(
                                    new FeederOpenLoop(mFeeder, -1.5),
                                    new ShooterOpenLoop(mShooter, -3))))
                    .andThen(new FeederOpenLoop(mFeeder, -2).withTimeout(0.08))));
  }
}

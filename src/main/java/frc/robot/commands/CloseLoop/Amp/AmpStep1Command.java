// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CloseLoop.Amp;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;
import frc.robot.commands.CloseLoop.pivot.SetPivotAngle;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpStep1Command extends SequentialCommandGroup {
  /** Creates a new AmpCommandGroup. */
  public AmpStep1Command(
      DriveSubsystem mDrive, ShooterPivotSubsystem mPivot, FeederSubsystem mFeeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
                () -> {
                  Robot.setRobotState(RobotState.AMP);
                })
            .andThen(new SetPivotAngle(mPivot, 60, true))
            .andThen(
                new InstantCommand(
                    () -> {
                      mFeeder.setButtonPressed(true);
                    })));
  }
}

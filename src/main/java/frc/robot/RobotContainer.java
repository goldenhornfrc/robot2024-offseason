// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot.RobotState;
import frc.robot.commands.CloseLoop.Amp.AmpStep1Command;
import frc.robot.commands.CloseLoop.Amp.AmpStep2Command;
import frc.robot.commands.CloseLoop.backIntake.BackIntakeCommandGroup;
import frc.robot.commands.CloseLoop.drive.DriveCommand;
import frc.robot.commands.CloseLoop.feeder.FeedWhenReady;
import frc.robot.commands.CloseLoop.frontIntake.FrontIntakeCommandGroup;
import frc.robot.commands.CloseLoop.pivot.SetPivotAngle;
import frc.robot.commands.CloseLoop.sensors.WaitForBackSensor;
import frc.robot.commands.CloseLoop.shooter.SetShooterRPM;
import frc.robot.commands.OpenLoop.BothIntakeOpenLoop;
import frc.robot.commands.OpenLoop.FeederOpenLoop;
import frc.robot.commands.OpenLoop.FrontIntakeOpenLoop;
import frc.robot.commands.OpenLoop.ShooterOpenLoop;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOFalcon;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveState;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOFalcon;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterPivot.ShooterPivotIO;
import frc.robot.subsystems.shooterPivot.ShooterPivotIOFalcon;
import frc.robot.subsystems.shooterPivot.ShooterPivotSubsystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller
  public static final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Subsystems
  // public static DriveSubsystem drive;
  public static IntakeSubsystem intake;
  public static ShooterSubsystem shooter;
  public static ShooterPivotSubsystem pivot;
  public static FeederSubsystem feeder;
  public static DriveSubsystem drive;
  // Dashboard inputs
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 3000.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        intake = new IntakeSubsystem(new IntakeIOSpark());
        shooter = new ShooterSubsystem(new ShooterIOKraken());
        pivot = new ShooterPivotSubsystem(new ShooterPivotIOFalcon());
        feeder = new FeederSubsystem(new FeederIOFalcon());
        drive =
            new DriveSubsystem(
                new DriveIOFalcon(
                    DriveConstants.kDriveTrain.getDriveTrainConstants(),
                    DriveConstants.kDriveTrain.getModuleConstants()));

        // Real robot, instantiate hardware IO implementations

        break;

      case SIM:
        intake = new IntakeSubsystem(new IntakeIOSpark()); // TODO
        shooter = new ShooterSubsystem(new ShooterIOKraken()); // TODO
        pivot = new ShooterPivotSubsystem(new ShooterPivotIOFalcon()); // TODO
        feeder = new FeederSubsystem(new FeederIOFalcon()); // TODO
        drive =
            new DriveSubsystem(
                new DriveIOFalcon(
                    DriveConstants.kDriveTrain.getDriveTrainConstants(),
                    DriveConstants.kDriveTrain.getModuleConstants()));
        // Sim robot, instantiate physics sim IO implementations

        break;

      default:
        intake = new IntakeSubsystem(new IntakeIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        pivot = new ShooterPivotSubsystem(new ShooterPivotIO() {});
        feeder = new FeederSubsystem(new FeederIO() {});
        drive = new DriveSubsystem(new DriveIO() {});
        // Replayed robot, disable IO implementations

        break;
    }

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        new DriveCommand(
            drive,
            () -> MathUtil.applyDeadband(controller.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(controller.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05)));

    controller.cross().onTrue(new SetPivotAngle(pivot, 60, true));
    controller.triangle().onTrue(new SetPivotAngle(pivot, 0, false));
    controller.circle().whileTrue(new FeederOpenLoop(feeder, 4));

    controller
        .square()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setTargetHeading(90);
                  drive.setDriveState(DriveState.HEADING_LOCK);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  drive.setDriveState(DriveState.OPEN_LOOP);
                }));

    controller
        .R1()
        .whileTrue(getBackIntakeGroupCommand())
        .onFalse(
            getIdleCommand()
                .alongWith(
                    new InstantCommand(
                        () -> {
                          drive.setDriveState(DriveState.OPEN_LOOP);
                        })));
    controller
        .L1()
        .whileTrue(getFrontIntakeGroupCommand())
        .onFalse(
            getIdleCommand()
                .alongWith(
                    new InstantCommand(
                        () -> {
                          drive.setDriveState(DriveState.OPEN_LOOP);
                        })));

    controller
        .povUp()
        .onTrue(
            new ConditionalCommand(
                new AmpStep2Command(feeder, shooter, drive, pivot).andThen(getIdleCommand()),
                new AmpStep1Command(drive, pivot, feeder),
                feeder::getButtonPress));

    controller.L2().whileTrue(new FeedWhenReady(drive, shooter, feeder, pivot));

    controller.R2().whileTrue(getSpeakerShot()).onFalse(getIdleCommand());

    controller.povDown().whileTrue(getUnstuckNoteCommand());

    controller.povLeft().whileTrue(getIntakesOuttake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  private Map<String, ChoreoTrajectory> loadTrajectories() {
    Set<String> trajNames;
    try {
      if (Robot.isReal()) {
        trajNames = listFilesUsingFilesList("/home/lvuser/deploy/choreo");
      } else {
        trajNames = listFilesUsingFilesList("src/main/deploy/choreo");
      }
    } catch (IOException e) {
      DriverStation.reportError("Invalid Directory! Trajectories failed to load!", true);
      return null;
    }
    return trajNames.stream()
        .collect(
            Collectors.toMap(
                entry -> entry.replace(".traj", ""),
                entry -> Choreo.getTrajectory(entry.replace(".traj", ""))));
  }

  private Set<String> listFilesUsingFilesList(String dir) throws IOException {
    try (Stream<Path> stream = Files.list(Paths.get(dir))) {
      return stream
          .filter(file -> !Files.isDirectory(file))
          .map(Path::getFileName)
          .map(Path::toString)
          .collect(Collectors.toSet());
    }
  }

  public Command getSpeakerShot() {
    return new SetPivotAngle(pivot, 40, true)
        .alongWith(new SetShooterRPM(shooter, 3500, 3500))
        .alongWith(
            new InstantCommand(
                () -> {
                  Robot.setRobotState(RobotState.SPEAKER);
                }));
  }

  public Command getIdleCommand() {
    return new SetPivotAngle(pivot, 0, false)
        .alongWith(
            new InstantCommand(
                () -> {
                  Robot.setRobotState(RobotState.IDLE);
                }));
  }

  public Command getIntakesOuttake() {
    return new BothIntakeOpenLoop(intake, -4, -4)
        .alongWith(new ShooterOpenLoop(shooter, 4))
        .alongWith(new FeederOpenLoop(feeder, -3));
  }

  public Command getBackIntakeGroupCommand() {
    return new BackIntakeCommandGroup(feeder, intake, shooter, pivot)
        .alongWith(
            new InstantCommand(
                () -> {
                  drive.setDriveState(DriveState.INTAKE_STATE);
                }));
  }

  public Command getFrontIntakeGroupCommand() {
    return new FrontIntakeCommandGroup(pivot, shooter, intake, feeder)
        .alongWith(
            new InstantCommand(
                () -> {
                  drive.setDriveState(DriveState.INTAKE_STATE);
                }));
  }

  public Command getUnstuckNoteCommand() {
    return new SetPivotAngle(pivot, 30, true)
        .andThen(
            new WaitForBackSensor(feeder)
                .raceWith(
                    Commands.parallel(
                        new ShooterOpenLoop(shooter, -8),
                        new FrontIntakeOpenLoop(intake, 4),
                        new FeederOpenLoop(feeder, -1.5)))
                .andThen(
                    new WaitUntilCommand(feeder::getFrontSensor)
                        .raceWith(
                            Commands.parallel(
                                new FeederOpenLoop(feeder, -1.5),
                                new ShooterOpenLoop(shooter, -3))))
                .andThen(new FeederOpenLoop(feeder, -2).withTimeout(0.08)));
  }
}

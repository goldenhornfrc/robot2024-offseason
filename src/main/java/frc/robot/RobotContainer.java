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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.CloseLoop.BackIntakeCommandGroup;
import frc.robot.commands.CloseLoop.DefaultDriveCommand;
import frc.robot.commands.CloseLoop.FrontIntakeCommandGroup;
import frc.robot.commands.CloseLoop.SetPivotAngle;
import frc.robot.commands.CloseLoop.SetShooterRPM;
import frc.robot.commands.OpenLoop.FeederOpenLoop;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOFalcon;
import frc.robot.subsystems.drive.DriveSubsystem;
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
        new DefaultDriveCommand(
            drive,
            () -> MathUtil.applyDeadband(controller.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(controller.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(controller.getRightX(), 0.05),
            () -> true));
    controller.cross().onTrue(new SetPivotAngle(pivot, 60, true));
    controller.triangle().onTrue(new SetPivotAngle(pivot, 0, false));
    controller.circle().whileTrue(new FeederOpenLoop(feeder, 4));
    controller
        .square()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setTargetHeading(50);
                }));
    controller.R1().whileTrue(new BackIntakeCommandGroup(feeder, intake, shooter, pivot));
    controller.L1().whileTrue(new FrontIntakeCommandGroup(pivot, shooter, intake, feeder));

    controller.L2().whileTrue(new SetShooterRPM(shooter, 4000, 4000));

    // controller.triangle().whileTrue(new FeederOpenLoop(feeder, 10));
    // controller.square().whileTrue(new ShooterPivotOpenLoop(pivot, -2));
    // controller.square().whileTrue(new SetShooterRPM(shooter, 3000, 3000));
    /*  controller
    .triangle()
    .whileTrue(
        new FrontIntakeOpenLoop(intake, 5)
            .alongWith(new ShooterOpenLoop(shooter, -5))
            .alongWith(new FeederOpenLoop(feeder, -2))); */
    // controller.y().whileTrue(new ShooterPivotOpenLoop(pivot, 3.5));
    // controller.x().whileTrue(new SetShooterRPM(shooter, 2000, 2000));
    // controller.b().whileTrue(new SetPivotAngle(pivot, 15));

    /*controller
        .button(1) // BACK INTAKE
        .whileTrue(
            new BackIntakeOpenLoop(intake, 3)
                .alongWith(new FeederOpenLoop(feeder, 3))
                .alongWith(new ShooterOpenLoop(shooter, 4)));

    controller
        .button(2) // FRONT INTAKE
        .whileTrue(
            new FrontIntakeOpenLoop(intake, 3)
                .alongWith(new FeederOpenLoop(feeder, 3))
                .alongWith(new ShooterOpenLoop(shooter, -4))); */
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
}

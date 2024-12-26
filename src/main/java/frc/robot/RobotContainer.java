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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot.RobotState;
import frc.robot.commands.CommandGroups.Amp.AmpStep1Command;
import frc.robot.commands.CommandGroups.Amp.AmpStep2Command;
import frc.robot.commands.CommandGroups.BothIntakeOpenLoop;
import frc.robot.commands.CommandGroups.object.ObjectDetection;
import frc.robot.commands.backIntake.BackIntakeAutoCommand;
import frc.robot.commands.backIntake.BackIntakeCommandGroup;
import frc.robot.commands.backIntake.BackIntakeOpenLoop;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.feeder.FeedWhenReady;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.frontIntake.FrontIntakeCommandGroup;
import frc.robot.commands.frontIntake.FrontIntakeOpenLoop;
import frc.robot.commands.pivot.SetPivotAngle;
import frc.robot.commands.pivot.SetPivotAngleDist;
import frc.robot.commands.pivot.ShooterPivotOpenLoop;
import frc.robot.commands.sensors.WaitForBackSensor;
import frc.robot.commands.shooter.SetShooterRPM;
import frc.robot.commands.shooter.ShooterOpenLoop;
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
import frc.robot.subsystems.vision.VisionIOHardware;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private SendableChooser<Command> autoChoice = new SendableChooser<Command>();
  // Controller
  public static final CommandPS5Controller controller = new CommandPS5Controller(0);
  public static final CommandPS5Controller operator = new CommandPS5Controller(1);

  // Subsystems
  // public static DriveSubsystem drive;
  public static IntakeSubsystem intake;
  public static ShooterSubsystem shooter;
  public static ShooterPivotSubsystem pivot;
  public static FeederSubsystem feeder;
  public static DriveSubsystem drive;
  public static VisionSubsystem vision;

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
        vision = new VisionSubsystem(new VisionIOHardware());
        // Real robot, instantiate hardware IO implementations

        break;

      case SIM:
        intake = new IntakeSubsystem(new IntakeIOSpark()); // TODO
        shooter = new ShooterSubsystem(new ShooterIOKraken()); // TODO
        pivot = new ShooterPivotSubsystem(new ShooterPivotIOFalcon()); // TODO
        feeder = new FeederSubsystem(new FeederIOFalcon()); // TOO
        drive =
            new DriveSubsystem(
                new DriveIOFalcon(
                    DriveConstants.kDriveTrain.getDriveTrainConstants(),
                    DriveConstants.kDriveTrain.getModuleConstants()));
        // Sim robot, instantiate physics sim IO implementations
        vision = new VisionSubsystem(new VisionIOHardware());
        break;

      default:
        intake = new IntakeSubsystem(new IntakeIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        pivot = new ShooterPivotSubsystem(new ShooterPivotIO() {});
        feeder = new FeederSubsystem(new FeederIO() {});
        drive = new DriveSubsystem(new DriveIO() {});
        // Replayed robot, disable IO implementations
        vision = new VisionSubsystem(new VisionIOHardware());
        break;
    }

    NamedCommands.registerCommand("BackIntake", new BackIntakeOpenLoop(intake, 8.0));
    NamedCommands.registerCommand("Feeder 5Volts", new FeederOpenLoop(feeder, 7.0));
    NamedCommands.registerCommand(
        "FeederInstant",
        new InstantCommand(
            () -> {
              feeder.setVoltage(7);
            }));
    NamedCommands.registerCommand(
        "FeederStop",
        new InstantCommand(
            () -> {
              feeder.setVoltage(0);
            }));
    NamedCommands.registerCommand(
        "Shooter 3700RPM",
        new InstantCommand(
            () -> {
              shooter.setTargetRPM(3700, 3700);
            }));
    NamedCommands.registerCommand("Pivot44.5", new SetPivotAngle(pivot, 44.5, true));
    NamedCommands.registerCommand("PivotAngle41", new SetPivotAngle(pivot, 41.0, true));
    NamedCommands.registerCommand("PivotAngle42", new SetPivotAngle(pivot, 42.3, true));
    NamedCommands.registerCommand(
        "ShooterStop", new InstantCommand(() -> shooter.setVoltage(0, 0)));
    NamedCommands.registerCommand("PivotDistance", new SetPivotAngleDist(pivot, vision, true));
    NamedCommands.registerCommand("PivotAngle38", new SetPivotAngle(pivot, 37.7, true));
    NamedCommands.registerCommand("-Feeder", new FeederOpenLoop(feeder, -3));
    NamedCommands.registerCommand(
        "Pivot30", new InstantCommand(() -> pivot.setMotionMagicAngle(30.0)));

    NamedCommands.registerCommand(
        "BackIntakeAuto", new BackIntakeAutoCommand(feeder, intake, shooter, pivot));
    NamedCommands.registerCommand(
        "IntakeCommandGroup", new BackIntakeCommandGroup(feeder, intake, shooter, pivot));

    NamedCommands.registerCommand(
        "Override True",
        new InstantCommand(
            () -> {
              drive.setWantsOverride(true);
            }));
    NamedCommands.registerCommand(
        "Override False",
        new InstantCommand(
            () -> {
              drive.setWantsOverride(false);
            }));
    autoChoice = AutoBuilder.buildAutoChooser();

    // 6791
    SmartDashboard.putData("OTO SECICI", autoChoice);
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
            vision,
            () -> MathUtil.applyDeadband(controller.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(controller.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(-controller.getRightX(), 0.05)));
    controller
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> {
                  pivot.testSum(1);
                }));
    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  pivot.testSum(-1);
                }));
    controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  pivot.testSum(0.5);
                }));
    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  pivot.testSum(-0.5);
                }));
    controller
        .circle()
        .whileTrue(
            new ConditionalCommand(
                new InstantCommand(), new ObjectDetection(drive, vision), feeder::getFrontSensor));

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
        .whileTrue(getFrontIntakeGroupCommand().andThen(new SetPivotAngle(pivot, 25.0, true)))
        .onFalse(
            new InstantCommand(
                () -> {
                  drive.setDriveState(DriveState.OPEN_LOOP);
                  Robot.setRobotState(RobotState.IDLE);
                }));

    controller
        .cross()
        .onTrue(
            new ConditionalCommand(
                new AmpStep2Command(feeder, shooter, drive, pivot)
                    .andThen(getIdleCommand())
                    .andThen(
                        new InstantCommand(
                            () -> {
                              drive.setDriveState(DriveState.OPEN_LOOP);
                            })),
                new AmpStep1Command(drive, pivot, feeder)
                    .andThen(
                        new InstantCommand(
                            () -> {
                              drive.setDriveState(DriveState.AMP_STATE);
                            })),
                feeder::getButtonPress));

    controller
        .R2()
        .whileTrue(
            new RepeatCommand(
                new ConditionalCommand(
                    getSpeakerShot()
                        .alongWith(
                            new WaitCommand(0.3)
                                .andThen(
                                    Commands.waitUntil(
                                            () ->
                                                Constants.kShootingParams.isShooterPivotAtSetpoint(
                                                    pivot.getShooterPivotAngle(),
                                                    pivot.getTargetAngle()))
                                        .andThen(
                                            new FeederOpenLoop(feeder, -3)
                                                .withTimeout(0.05)
                                                .andThen(
                                                    new FeedWhenReady(
                                                        drive, shooter, feeder, pivot))))),
                    new InstantCommand(
                        () -> {
                          if (drive.getDriveState() != DriveState.VISION_STATE) {
                            drive.setDriveState(DriveState.HEADING_LOCK);
                            drive.setTargetHeading(
                                0.0); // TODO: Set to proper angles for each alliance. 0deg,
                            // 180deg
                          }
                        }),
                    () -> vision.getTargetInfo().targetValid)))
        .onFalse(
            getIdleCommand()
                .alongWith(new InstantCommand(() -> drive.setDriveState(DriveState.OPEN_LOOP))));

    controller
        .L2()
        .whileTrue(getFeedOverStageCommand())
        .onFalse(
            new InstantCommand(
                    () -> {
                      drive.setDriveState(DriveState.OPEN_LOOP);
                    })
                .alongWith(getIdleCommand()));

    // controller.povLeft().onTrue(new InstantCommand(() -> drive.seedFieldRelative()));

    operator
        .cross()
        .whileTrue(
            new SetPivotAngle(pivot, 45, true)
                .andThen(
                    new FeederOpenLoop(feeder, -6).alongWith(new BackIntakeOpenLoop(intake, -5))));
    operator.L1().whileTrue(new FrontIntakeOpenLoop(intake, -4));
    operator
        .L2()
        .whileTrue(
            new ShooterOpenLoop(shooter, 5.5).alongWith(new FrontIntakeOpenLoop(intake, -5.5)));
    operator.povDown().whileTrue(new ShooterPivotOpenLoop(pivot, -2));
    operator.options().onTrue(new InstantCommand(() -> pivot.resetEncoder()));
    operator
        .R2()
        .whileTrue(
            new SetPivotAngle(pivot, 60, true)
                .andThen(new FeederOpenLoop(feeder, -3).withTimeout(0.05))
                .andThen(new WaitCommand(1.0).alongWith(new FeederOpenLoop(feeder, 6)))
                .alongWith(new ShooterOpenLoop(shooter, 7)));

    new Trigger(intake::getShouldBlink).onTrue(vision.blinkTagLimelight());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getSpeakerShot() {
    return new SetPivotAngleDist(pivot, vision, true)
        .alongWith(new SetShooterRPM(shooter, 3700, 3700))
        .alongWith(
            new InstantCommand(
                () -> {
                  Robot.setRobotState(RobotState.SPEAKER);
                  drive.setDriveState(DriveState.VISION_STATE);
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

  public Command getFeedOverStageCommand() {
    return new SetPivotAngle(pivot, 50.0, true)
        .alongWith(
            new InstantCommand(
                () -> {
                  drive.setDriveState(DriveState.HEADING_LOCK);
                  drive.setTargetHeading(
                      Robot.getAlliance() == Alliance.Red
                          ? 30
                          : 210); // TODO: Set to proper angles for each alliance. 30deg, 150deg
                }))
        .alongWith(new SetShooterRPM(shooter, 3000, 3000))
        .alongWith(
            new WaitCommand(0.85)
                .andThen(new FeederOpenLoop(feeder, -3).withTimeout(0.05))
                .andThen(new FeederOpenLoop(feeder, 7)));
  }

  public Command getAutonomousCommand() {
    return autoChoice.getSelected();
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

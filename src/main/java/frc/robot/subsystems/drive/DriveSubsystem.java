package frc.robot.subsystems.drive;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.swerve.*;
import frc.lib.util.swerve.SwerveModule.DriveRequestType;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
  DriveIO io;

  DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  public final SwerveRequest.FieldCentricFacingAngle fieldCentricHeadingLockRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.ApplyChassisSpeeds autoRequest =
      new SwerveRequest.ApplyChassisSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("MyRobotPose", Pose2d.struct).publish();

  private final PhoenixPIDController visionController = new PhoenixPIDController(7.0, 0, 0.007);

  private boolean wantsOverride = false;

  public enum DriveState {
    OPEN_LOOP,
    HEADING_LOCK,
    INTAKE_STATE,
    VISION_STATE,
    AMP_STATE
  }

  private DriveState driveState = DriveState.OPEN_LOOP;

  private double targetHeading = 0.0;
  private boolean wantsHeadingLock = false;
  public RobotState robotState;

  public DriveSubsystem(DriveIO io) {
    this.io = io;

    fieldCentricHeadingLockRequest.HeadingController = new PhoenixPIDController(5.0, 0, 0.0);
    fieldCentricHeadingLockRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    fieldCentricHeadingLockRequest.HeadingController.setTolerance(Math.PI / 180.0);

    configurePathPlanner();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Heading", targetHeading);
    SmartDashboard.putString("Pose", getPose().toString());

    publisher.set(getPose());
    var targets = io.getModuleTargets();
    var states = io.getModuleStates();
    SmartDashboard.putNumberArray(
        "DesiredStates",
        new double[] {
          targets[0].angle.getDegrees(),
          targets[0].speedMetersPerSecond,
          targets[1].angle.getDegrees(),
          targets[1].speedMetersPerSecond,
          targets[2].angle.getDegrees(),
          targets[2].speedMetersPerSecond,
          targets[3].angle.getDegrees(),
          targets[3].speedMetersPerSecond
        });

    SmartDashboard.putNumberArray(
        "CurrentStates",
        new double[] {
          states[0].angle.getDegrees(),
          states[0].speedMetersPerSecond,
          states[1].angle.getDegrees(),
          states[1].speedMetersPerSecond,
          states[2].angle.getDegrees(),
          states[2].speedMetersPerSecond,
          states[3].angle.getDegrees(),
          states[3].speedMetersPerSecond
        });
  }

  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : getModuleLocations()) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0, 0),
            new PIDConstants(1.6, 0, 0),
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () -> false, // Robot.getAlliance() == Alliance.Red,
        this);

    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          Logger.recordOutput("PathPlanner/targetPose", pose);
        });

    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          Logger.recordOutput("PathPlanner/currentPose", pose);
        });

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    if (wantsOverride) {
      var info = RobotContainer.vision.getTargetInfo();

      if (info.targetValid) {
        return Robot.getAlliance() == Alliance.Red ? Optional.of(getPose().getRotation().minus(Rotation2d.fromDegrees(info.targetTx))) : Optional.of(getPose().getRotation().plus(Rotation2d.fromDegrees(info.targetTx))) ;
      } else {
        return Optional.empty();
      }
    } else {
      return Optional.empty();
    }
  }

  public Command getPathFollowCommand(ChoreoTrajectory trajectory, boolean resetPose) {
    return Commands.sequence(
        new InstantCommand(
            () -> {
              if (resetPose) {
                this.seedFieldRelative(trajectory.getInitialPose());
                ;
              }
            }),
        Choreo.choreoSwerveCommand(
            trajectory, //
            this::getPose, //
            new PIDController(1.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0),
            (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
            () -> false, //
            this));
  }

  public Translation2d[] getModuleLocations() {
    return io.getModuleLocations();
  }

  public void setWantsOverride(boolean WantsOVERRIDE) {
    wantsOverride = WantsOVERRIDE;
  }

  public void setControl(SwerveRequest request) {

    io.setControl(request);
  }

  // API
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return io.applyRequest(requestSupplier, this).withName("Swerve drive request");
  }

  public void seedFieldRelative() {
    io.seedFieldRelative();
  }

  public PhoenixPIDController getVisionController() {
    return visionController;
  }

  public void seedFieldRelative(Pose2d location) {
    io.seedFieldRelative(location);
  }

  @AutoLogOutput(key = "getTargetheading")
  public double getTargetHeading() {
    return targetHeading;
  }

  public void setTargetHeading(double heading) {
    targetHeading = heading;
  }

  @AutoLogOutput(key = "getWantsHeadingLock")
  public boolean getWantsHeadingLock() {
    return wantsHeadingLock;
  }

  public void setWantsHeadingLock(boolean newWantsHeadingLock) {
    wantsHeadingLock = newWantsHeadingLock;
  }

  public void setDriveState(DriveState state) {
    driveState = state;
  }

  public DriveState getDriveState() {
    return driveState;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return io.getChassisSpeeds();
  }

  public Pose2d getPose() {
    return io.getPose();
  }

  public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
    if (Robot.isReal()) {
      io.addVisionMeasurement(visionFieldPoseEstimate);
    }
  }
}

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team254.lib.pathplanner.AdvancedAutoBuilder;
import com.team254.lib.pathplanner.AdvancedHolonomicPathFollowerConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.util.swerve.*;
import frc.robot.util.swerve.SwerveModule.DriveRequestType;
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


  public enum DriveState {
    OPEN_LOOP,
    HEADING_LOCK
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
  }

  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : getModuleLocations()) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AdvancedAutoBuilder.configureHolonomic(
        null,
        (pose) -> {}, // TODO FİXXXXXXXXXXXXxx
        // this::seedFieldRelative,
        null,
        (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
        new AdvancedHolonomicPathFollowerConfig(
            new PIDConstants(0, 0, 0),
            new PIDConstants(0, 0, 0),
            0,
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().equals(Optional.of(Alliance.Red)),
        this);

    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          Logger.recordOutput("PathPlanner/targetPose", pose);
        });

    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          Logger.recordOutput("PathPlanner/currentPose", pose);
        });
  }

  public Translation2d[] getModuleLocations() {
    return io.getModuleLocations();
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
    return io.getKinematics().toChassisSpeeds(inputs.ModuleStates);
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

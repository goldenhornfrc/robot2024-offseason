package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.RobotTime;
import frc.lib.util.swerve.*;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** DriveIOHardware offers a CTRE SwerveDrivetrain in the interface shape of a DriveIO */
public class DriveIOFalcon extends SwerveDrivetrain implements DriveIO {

  AtomicReference<SwerveDriveState> telemetryCache_ = new AtomicReference<>();

  private StatusSignal<Double> angularPitchVelocity;
  private StatusSignal<Double> angularRollVelocity;
  private StatusSignal<Double> angularYawVelocity;
  private StatusSignal<Double> roll;
  private StatusSignal<Double> pitch;
  private StatusSignal<Double> accelerationX;
  private StatusSignal<Double> accelerationY;

  private SwerveDriveState state;

  public DriveIOFalcon(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    angularPitchVelocity = m_pigeon2.getAngularVelocityYDevice();
    angularRollVelocity = m_pigeon2.getAngularVelocityXDevice();
    angularYawVelocity = m_pigeon2.getAngularVelocityZDevice();
    roll = m_pigeon2.getRoll();
    pitch = m_pigeon2.getPitch();
    accelerationX = m_pigeon2.getAccelerationX();
    accelerationY = m_pigeon2.getAccelerationY();

    BaseStatusSignal.setUpdateFrequencyForAll(
        250,
        angularPitchVelocity,
        angularRollVelocity,
        angularYawVelocity,
        roll,
        pitch,
        accelerationX,
        accelerationY);

    registerTelemetry(telemetryConsumer_);
  }

  public void resetOdometry(Pose2d pose) {
    super.seedFieldRelative(pose);
  }

  public Translation2d[] getModuleLocations() {
    return m_moduleLocations;
  }

  public void setControl(SwerveRequest request) {
    super.setControl(request);
  }

  public Command applyRequest(
      Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
    return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemRequired);
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return super.getState().speeds;
  }

  public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
    if (visionFieldPoseEstimate.getVisionMeasurementStdDevs() == null) {
      this.addVisionMeasurement(
          visionFieldPoseEstimate.getVisionRobotPoseMeters(),
          visionFieldPoseEstimate.getTimestampSeconds());
    } else {
      this.addVisionMeasurement(
          visionFieldPoseEstimate.getVisionRobotPoseMeters(),
          visionFieldPoseEstimate.getTimestampSeconds(),
          visionFieldPoseEstimate.getVisionMeasurementStdDevs());
    }
  }

  public Pose2d getPose() {

    return super.getState().Pose;
  }

  public SwerveModuleState[] getModuleTargets() {
    return super.getState().ModuleTargets;
  }

  public SwerveModuleState[] getModuleStates() {
    return super.getState().ModuleStates;
  }

  Consumer<SwerveDriveState> telemetryConsumer_ =
      swerveDriveState -> {
        double timestamp = RobotTime.getTimestampSeconds();
        // telemetryCache_.set(swerveDriveState.clone());
      };

  @Override
  public void readInputs(DriveIOInputs inputs) {

    inputs.fromSwerveDriveState(telemetryCache_.get());
    var gyroRotation = inputs.Pose.getRotation();
    inputs.gyroAngle = gyroRotation.getDegrees();
    var measuredRobotRelativeChassisSpeeds = m_kinematics.toChassisSpeeds(inputs.ModuleStates);
    var measuredFieldRelativeChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobotRelativeChassisSpeeds, gyroRotation);
    var desiredFieldRelativeChassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            m_kinematics.toChassisSpeeds(inputs.ModuleTargets), gyroRotation);

    BaseStatusSignal.refreshAll(
        angularRollVelocity,
        angularPitchVelocity,
        angularYawVelocity,
        pitch,
        roll,
        accelerationX,
        accelerationY);

    double timestamp = RobotTime.getTimestampSeconds();
    double rollRadsPerS = Units.degreesToRadians(angularRollVelocity.getValueAsDouble());
    double pitchRadsPerS = Units.degreesToRadians(angularPitchVelocity.getValueAsDouble());
    double yawRadsPerS = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());
    // Trust gyro rate more than odometry.
    var fusedFieldRelativeChassisSpeeds =
        new ChassisSpeeds(
            measuredFieldRelativeChassisSpeeds.vxMetersPerSecond,
            measuredFieldRelativeChassisSpeeds.vyMetersPerSecond,
            yawRadsPerS);

    double pitchRads = Units.degreesToRadians(pitch.getValueAsDouble());
    double rollRads = Units.degreesToRadians(roll.getValueAsDouble());
    double accelX = accelerationX.getValueAsDouble();
    double accelY = accelerationY.getValueAsDouble();

    /* robotState_.addDriveMotionMeasurements(timestamp, rollRadsPerS, pitchRadsPerS, yawRadsPerS,
    pitchRads, rollRads, accelX, accelY, desiredFieldRelativeChassisSpeeds,
    measuredRobotRelativeChassisSpeeds, measuredFieldRelativeChassisSpeeds,
    fusedFieldRelativeChassisSpeeds); */
  }

  @Override
  public void logModules(SwerveDrivetrain.SwerveDriveState driveState) {
    final String[] moduleNames = {"Drive/FL", "Drive/FR", "Drive/BL", "Drive/BR"};
    for (int i = 0; i < ModuleCount; i++) {
      Logger.recordOutput(
          moduleNames[i] + " Absolute Encoder Angle",
          Modules[i].getCANcoder().getAbsolutePosition().getValueAsDouble() * 360);
      Logger.recordOutput(moduleNames[i] + " Steering Angle", driveState.ModuleStates[i].angle);
      Logger.recordOutput(
          moduleNames[i] + " Target Steering Angle", driveState.ModuleTargets[i].angle);
      Logger.recordOutput(
          moduleNames[i] + " Drive Velocity", driveState.ModuleStates[i].speedMetersPerSecond);
      Logger.recordOutput(
          moduleNames[i] + " Target Drive Velocity",
          driveState.ModuleTargets[i].speedMetersPerSecond);
    }
  }
}

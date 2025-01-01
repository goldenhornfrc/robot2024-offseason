package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.MathHelpers;
import frc.lib.util.swerve.SwerveDrivetrain;
import frc.lib.util.swerve.SwerveRequest;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {

  @AutoLog
  class DriveIOInputs extends SwerveDrivetrain.SwerveDriveState {
    public double gyroAngle = 0.0;

    DriveIOInputs() {
      this.Pose = MathHelpers.kPose2dZero;
    }

    public void fromSwerveDriveState(SwerveDrivetrain.SwerveDriveState stateIn) {
      this.Pose = stateIn.Pose;
      this.SuccessfulDaqs = stateIn.SuccessfulDaqs;
      this.FailedDaqs = stateIn.FailedDaqs;
      this.ModuleStates = stateIn.ModuleStates;
      this.ModuleTargets = stateIn.ModuleTargets;
      this.speeds = stateIn.speeds;
      this.OdometryPeriod = stateIn.OdometryPeriod;
    }
  }

  default void readInputs(DriveIOInputs inputs) {}

  public default void logModules(SwerveDrivetrain.SwerveDriveState driveState) {}

  public default void seedFieldRelative() {}

  public default void seedFieldRelative(Pose2d location) {}

  public default void resetOdometry(Pose2d pose) {}

  public default Translation2d[] getModuleLocations() {
    return new Translation2d[] {};
  }

  public default SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(getModuleLocations());
  }

  public default Pose2d getPose() {
    return new Pose2d();
  }

  public default SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {};
  }

  public default SwerveModuleState[] getModuleTargets() {
    return new SwerveModuleState[] {};
  }

  public default void setControl(frc.lib.util.swerve.SwerveRequest request) {}

  public default Command applyRequest(
      Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
    return new Command() {};
  }

  public default ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds();
  }

  public default void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {}
}

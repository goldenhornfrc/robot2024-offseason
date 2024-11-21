package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.lib.Conversions;
import frc.robot.util.swerve.SwerveDrivetrainConstants;
import frc.robot.util.swerve.SwerveModule.ClosedLoopOutputType;
import frc.robot.util.swerve.SwerveModuleConstants;
import frc.robot.util.swerve.SwerveModuleConstants.SteerFeedbackType;
import frc.robot.util.swerve.SwerveModuleConstantsFactory;

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
public class TunerConstants {
  // Both sets of gains need to be tuned to your individual robot.

  public static final double wheelDiameter = Units.inchesToMeters(3.9);
  public static final double wheelCircumference = wheelDiameter * Math.PI;
  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(60).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
  // When using closed-loop control, the drive motor uses the control
  private static final Slot0Configs driveGains =
      new Slot0Configs()
          .withKP(0.17)
          .withKI(0)
          .withKD(0)
          .withKS(0)
          .withKV(
              12.0
                  / Conversions.MPSToRPS(
                      DriveConstants.kMaxVel, wheelCircumference, 6.746031746031747))
          .withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the  PID/FF gains for the steer motors
  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final double kSlipCurrentA = 80.0;

  // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static final Pigeon2Configuration pigeonConfigs = null;

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  public static final double kSpeedAt12VoltsMps = 5.21;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 3.5714285714285716;

  private static final double kDriveGearRatio = 6.122448979591837;
  private static final double kSteerGearRatio = 12.8;
  private static final double kWheelRadiusInches = 2;

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final String kCANbusName = "Canivore";
  private static final int kPigeonId = 0;

  // These are only used for simulation
  private static final double kSteerInertia = 0.00001;
  private static final double kDriveInertia = 0.001;
  // Simulated voltage necessary to overcome friction
  private static final double kSteerFrictionVoltage = 0.25;
  private static final double kDriveFrictionVoltage = 0.25;

  private static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants().withCANbusName(kCANbusName).withPigeon2Id(kPigeonId);

  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage)
          .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withCANcoderInitialConfigs(cancoderInitialConfigs);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 2;
  private static final int kFrontLeftSteerMotorId = 1;
  private static final int kFrontLeftEncoderId = 9;
  private static final double kFrontLeftEncoderOffset = 0.47265625;
  private static final boolean kFrontLeftSteerInvert = false;

  private static final double kFrontLeftXPosInches = 6.9905;
  private static final double kFrontLeftYPosInches = 11.6285;

  // Front Right
  private static final int kFrontRightDriveMotorId = 4;
  private static final int kFrontRightSteerMotorId = 3;
  private static final int kFrontRightEncoderId = 14;
  private static final double kFrontRightEncoderOffset = -0.275390625;
  private static final boolean kFrontRightSteerInvert = false;

  private static final double kFrontRightXPosInches = 6.9905;
  private static final double kFrontRightYPosInches = -11.6285;

  // Back Left
  private static final int kBackLeftDriveMotorId = 6;
  private static final int kBackLeftSteerMotorId = 5;
  private static final int kBackLeftEncoderId = 15;
  private static final double kBackLeftEncoderOffset = -0.36181640625;
  private static final boolean kBackLeftSteerInvert = false;

  private static final double kBackLeftXPosInches = -6.9905;
  private static final double kBackLeftYPosInches = 11.6285;

  // Back Right
  private static final int kBackRightDriveMotorId = 8;
  private static final int kBackRightSteerMotorId = 7;
  private static final int kBackRightEncoderId = 11;
  private static final double kBackRightEncoderOffset = 0.289794921875;
  private static final boolean kBackRightSteerInvert = false;

  private static final double kBackRightXPosInches = -6.9905;
  private static final double kBackRightYPosInches = -11.6285;

  private static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              Units.inchesToMeters(kFrontLeftXPosInches),
              Units.inchesToMeters(kFrontLeftYPosInches),
              kInvertLeftSide)
          .withSteerMotorInverted(kFrontLeftSteerInvert);
  private static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              Units.inchesToMeters(kFrontRightXPosInches),
              Units.inchesToMeters(kFrontRightYPosInches),
              kInvertRightSide)
          .withSteerMotorInverted(kFrontRightSteerInvert);
  private static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              Units.inchesToMeters(kBackLeftXPosInches),
              Units.inchesToMeters(kBackLeftYPosInches),
              kInvertLeftSide)
          .withSteerMotorInverted(kBackLeftSteerInvert);
  private static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              Units.inchesToMeters(kBackRightXPosInches),
              Units.inchesToMeters(kBackRightYPosInches),
              kInvertRightSide)
          .withSteerMotorInverted(kBackRightSteerInvert);

  public static final CommandSwerveDrivetrain DriveTrain =
      new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
}

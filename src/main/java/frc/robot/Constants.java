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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.InterpolatingTreeMap;
import frc.robot.lib.ShootingParameters;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class AutoConstants {
    public static final double kPThetaController = 0;
    public static final double kPXController = 0;
  }

  public static class ShooterPivotConstants {
    public static final double kGearRatio = 1 / 144;
    public static final double kS = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.0;
    public static final double kG = 0.0;
    public static final double velocitySetpoint = 0;
    public static final ArmFeedforward ff =
        new ArmFeedforward(
            ShooterPivotConstants.kS,
            ShooterPivotConstants.kG,
            ShooterPivotConstants.kV,
            ShooterPivotConstants.kA);
  }

  public static class FieldConstants {
    public static final Translation2d BlueSpeaker = new Translation2d(0.0, 5.55);
    public static final Translation2d RedSpeaker = new Translation2d(16.54, 5.55);
  }

  public static class ShooterConstants {
    public static final int leftMotorID = 42;
    public static final int rightMotorID = 41;
  }

  public static class DriveConstants {
    public static final double kMaxVel = 5;
    public static final double kMaxAngularVel = Units.degreesToRadians(360 * 1.15);
    public static final double kSteerJoystickDeadband = 0.05;
    public static final double kHeadingControllerP = 3.5;
    public static final double kHeadingControllerI = 0.0;
    public static final double kHeadingControllerD = 0.0;

    public static final CommandSwerveDrivetrain kDriveTrain = TunerConstants.DriveTrain;
  }

  public static boolean kIsReplay = false;
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static double[][] kRPMValues = {
    {7.41, 6500.0},
    {5.31, 4000.0},
    {4.81, 4000.0},
    {4.31, 4000.0},
    {3.81, 4000.0},
    {3.31, 4000.0},
    {2.81, 4000.0},
    {2.31, 4000.0},
    {1.81, 4000.0},
    {1.31, 4000.0},
    {0.0, 0.0}
  };

  public static double[][] kPivotValues = {
    {1.31, 60.0},
    {0.0, 60.0}
  };

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPivotMap =
      new InterpolatingTreeMap<>();
  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap =
      new InterpolatingTreeMap<>();

  static {
    for (double[] pair : kRPMValues) {
      kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }

    for (double[] pair : kPivotValues) {
      kPivotMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
  }

  public static final ShootingParameters kShootingParams =
      new ShootingParameters(
          kPivotMap,
          kRPMMap, // rpm map
          60.0, // shooter allowable error (rpm)
          1.0 // pivot allowable error (°)
          );
}

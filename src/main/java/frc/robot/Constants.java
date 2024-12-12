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
    public static final double kGearRatio = 1.0 / 144.0;
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

  public static double[][] kPivotValues = {
    {109.0, 60.0},
    {135.0, 58.0},
    {160.0, 56.0},
    {185.0, 53.0},
    {210.0, 50.0},
    {237.0, 48.0},
    {260.0, 46.0},
    {284.0, 44.0},
    {310.0, 43.0},
    {335.0, 42.0},
    {360.0, 41.0},
    {385.0, 41.0},
    {410.0, 40.0},
    {435.0, 39.0},
    {460.0, 39.0},
    {472.0, 39.0},
    {500.0, 39.0}
  };

  public static double[][] kTYDistValuesRed = {
    {19.27, 109.0},
    {17.4, 116.0},
    {15.58, 123.8},
    {13.55, 131.7},
    {11.48, 141.3},
    {9.72, 149.3},
    {8.45, 157.3},
    {7.23, 164.3},
    {6.24, 169.3},
    {5.1, 177.3},
    {3.84, 184.3},
    {2.65, 194.3},
    {1.8, 201.3},
    {0.82, 209.3},
    {-0.25, 217.3},
    {-0.78, 224.3},
    {-1.65, 231.3},
    {-2.53, 239.3},
    {-3.24, 246.3},
    {-3.6, 251.3},
    {-4.63, 266.3},
    {-5.39, 271.3},
    {-6.16, 283.3},
    {-6.83, 293.3},
    {-7.4, 301.3},
    {-7.98, 309.3},
    {-8.6, 319.3},
    {-9.05, 329.3},
    {-9.6, 339.3},
    {-10.14, 349.3},
    {-10.52, 359.3},
    {-11.1, 369.3},
    {-11.62, 379.3},
    {-11.9, 389.3},
    {-12.29, 399.3},
    {-12.77, 409.3},
    {-13.11, 419.3},
    {-13.58, 429.3},
    {-14.04, 439.3},
    {-14.39, 449.3},
    {-14.57, 459.3},
    {-14.89, 469.3},
    {-15.16, 479.3},
    {-15.34, 489.3}
  };

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPivotMap =
      new InterpolatingTreeMap<>();

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceMap =
      new InterpolatingTreeMap<>();

  static {
    for (double[] pair : kPivotValues) {
      kPivotMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }

    for (double[] pair : kTYDistValuesRed) {
      kDistanceMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
  }

  public static final ShootingParameters kShootingParams =
      new ShootingParameters(
          kPivotMap,
          80.0, // shooter allowable error (rpm)
          0.35 // pivot allowable error (°)
          );
}

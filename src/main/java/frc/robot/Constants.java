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
import frc.lib.InterpolatingDouble;
import frc.lib.InterpolatingTreeMap;
import frc.lib.ShootingParameters;
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
    {109.0, 60.3},
    {135.0, 58.3},
    {166.0, 52.3},
    {178.0, 49.8},
    {193.0, 48.8},
    {214.0, 46.75},
    {229.5, 44.75},
    {246.5, 43.75},
    {272.0, 42.3},
    {287.0, 41.8},
    {300, 41.3},
    {315.5, 39.8},
    {338.0, 39.65},
    {362.0, 39.3},
    {365.0, 38.8},
    {378.0, 38.3},
    {384.0, 38.0},
    {400.0, 37.3},
    {430.0, 36.3},
    {450.0, 36.1}
  };

  public static double[][] kTYDistValuesRed = {
    {18.74, 106.0},
    {17.02, 113.5},
    {15.90, 118.0},
    {15.18, 122.0},
    {14.52, 126.0},
    {13.58, 131.0},
    {12.73, 135.0},
    {11.33, 141.0},
    {10.12, 148.5},
    {9.64, 151.6},
    {8.98, 154.6},
    {7.53, 164.7},
    {6.74, 169.7},
    {5.66, 177.0},
    {3.94, 190.3},
    {2.97, 199.3},
    {2.25, 204.3},
    {1.52, 211.0},
    {0.50, 219.3},
    {0.0, 223.0},
    {-0.78, 230.0},
    {-1.50, 237.0},
    {-2.30, 245.0},
    {-2.95, 252.0},
    {-3.88, 262.7},
    {-4.90, 276.0},
    {-6.40, 295.0},
    {-7.05, 306.0},
    {-7.38, 312.0},
    {-7.74, 318.0},
    {-8.26, 327.5},
    {-8.74, 336.0},
    {-9.15, 346.0},
    {-9.50, 352.5},
    {-10.0, 364.0},
    {-10.48, 373.3},
    {-10.89, 383.0},
    {-11.18, 388.0},
    {-11.39, 393.0},
    {-11.76, 400.0},
    {-11.96, 405.3},
    {-12.4, 417.0},
    {-12.92, 428.5},
    {-13.20, 436.0},
    {-13.66, 448.3},
    {-13.78, 454.0},
    {-13.95, 462.0},
    {-14.3, 473.5},
    {-14.45, 482.5},
    {-14.64, 492.0},
    {-14.83, 502.944},
    {-18.55, 593.379}
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

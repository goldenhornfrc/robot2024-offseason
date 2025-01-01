// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

/** Add your docs here. */
public class Conversions {

  public static double rotationsToDegrees(double rotations, double gearRatio) {
    return (rotations / gearRatio) * 360.0;
  }

  public static double rotationsToDegrees(double rotations) {
    return (rotations) * 360.0;
  }

  public static double degreesToRotations(double degrees, double gearRatio) {
    return (degrees / 360.0) * gearRatio;
  }

  public static double degreesToRotations(double degrees) {
    return (degrees / 360.0);
  }

  public static double RPStoRPM(double rps, double gearRatio) {
    return (rps * 60.0) / gearRatio;
  }

  public static double RPStoRPM(double rps) {
    return (rps * 60.0);
  }

  public static double RPMtoRPS(double rpm, double gearRatio) {
    return (rpm / 60.0) * gearRatio;
  }

  public static double RPMtoRPS(double rpm) {
    return (rpm / 60.0);
  }

  public static double MPSToRPS(double velocity, double circumference, double gearRatio) {
    double wheelRPS = ((velocity) / circumference);
    double falconRPS = wheelRPS * gearRatio;
    return falconRPS;
  }

  public static double RPSToMPS(double rotationsPerSecond, double circumference, double gearRatio) {
    double wheelRPM = (rotationsPerSecond / gearRatio);
    double mps = wheelRPM * circumference;
    return mps;
  }
}

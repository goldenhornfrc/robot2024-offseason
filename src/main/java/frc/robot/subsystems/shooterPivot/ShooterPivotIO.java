// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterPivot;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterPivotIO {

  @AutoLog
  public static class ShooterPivotIOInputs {
    public double shooterPosition = 0;
    public double pivotVolts = 0;
  }

  public default void setVoltage(double voltage) {}

  public default void setPivotAngle(double targetAngle) {}

  public default double getShooterPivotAngle() {
    return 0;
  }

  public default double getTargetAngle() {
    return 0;
  }
  public default void updateInputs(ShooterPivotIOInputs inputs) {}
}
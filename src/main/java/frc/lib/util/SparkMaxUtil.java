// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Add your docs here. */
public class SparkMaxUtil {

  private static final int MAX_ATTEMPTS = 5;
  private static final double APPLY_PARAMETER_WAIT_TIME = 0.1;
  private static final double BURN_FLASH_WAIT_TIME = 0.5;

  /**
   * Attempt to apply parameter and check if specified parameter is set correctly
   *
   * @param parameterSetter Method to set desired parameter
   * @param parameterCheckSupplier Method to check for parameter in question
   * @return {@link REVLibError#kOk} if successful
   */
  public static REVLibError applyParameter(
      Supplier<REVLibError> parameterSetter,
      BooleanSupplier parameterCheckSupplier,
      String errorMessage) {
    if (RobotBase.isSimulation()) return parameterSetter.get();

    REVLibError status = REVLibError.kError;
    for (int i = 0; i < MAX_ATTEMPTS; i++) {
      status = parameterSetter.get();
      if (parameterCheckSupplier.getAsBoolean() && status == REVLibError.kOk) break;
      Timer.delay(APPLY_PARAMETER_WAIT_TIME);
    }

    checkStatus(status, errorMessage);
    return status;
  }

  /**
   * Check status and print error message if necessary
   *
   * @param status Status to check
   * @param errorMessage Error message to print
   */
  private static void checkStatus(REVLibError status, String errorMessage) {
    if (status != REVLibError.kOk) DriverStation.reportError(errorMessage, false);

    String.join(" ", "Error Configuring Spark MAX", errorMessage, "-", status.toString());
  }

  /**
   * Writes all settings to flash
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public static REVLibError burnFlash(CANSparkMax sparkMax) {
    if (RobotBase.isSimulation()) return REVLibError.kOk;

    Timer.delay(BURN_FLASH_WAIT_TIME);
    REVLibError status = sparkMax.burnFlash();
    Timer.delay(BURN_FLASH_WAIT_TIME);

    return status;
  }

  /**
   * Restore motor controller parameters to factory defaults until the next controller reboot
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public static REVLibError restoreFactoryDefaults(CANSparkMax sparkMax) {
    REVLibError status;
    status =
        applyParameter(
            () -> sparkMax.restoreFactoryDefaults(),
            () -> true,
            "Restore factory defaults failure! - ID : " + sparkMax.getDeviceId());

    return status;
  }
}

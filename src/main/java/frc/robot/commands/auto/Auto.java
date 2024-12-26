// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Map;

/** Add your docs here. */
public class Auto {

  public static Command testAuto(Map<String, ChoreoTrajectory> trajMap, DriveSubsystem drive) {
    return drive.getPathFollowCommand(trajMap.get("a"), true);
  }
}

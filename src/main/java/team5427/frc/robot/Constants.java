// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5427.frc.robot;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kCanivoreBusName = "canivore_bus_name";
  public static final double kOdometryFrequency =
      new CANBus(Constants.kCanivoreBusName).isNetworkFD() ? 250.0 : 100.0;

  public static final Frequency kHighPriorityUpdateFrequency = Hertz.of(100.0);
  public static final Frequency kMediumPriorityUpdateFrequency = Hertz.of(50.0);
  public static final Frequency kLowPriorityUpdateFrequency = Hertz.of(10.0);

  public static Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ModeTriggers {
    public static final Trigger kReal =
        new Trigger(
            () -> {
              return currentMode.equals(Mode.REAL);
            });
    public static final Trigger kSim =
        new Trigger(
            () -> {
              return currentMode.equals(Mode.SIM);
            });
    public static final Trigger kReplay =
        new Trigger(
            () -> {
              return currentMode.equals(Mode.REPLAY);
            });
  }

  public static final double kLoopSpeed = Units.millisecondsToSeconds(20);

  public static final boolean kIsTuningMode = true;

  public static RobotConfig config;

  public static class DriverConstants {
    public static final int kDriverJoystickPort = 0;
    public static final int kOperatorJoystickPort = 1;
    public static final double kDriverControllerJoystickDeadzone = 0.0;
    public static final double kDriverControllerRotationalControlJoystickDeadzone = 0.05;
  }
}

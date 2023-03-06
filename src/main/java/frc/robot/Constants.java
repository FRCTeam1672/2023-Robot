// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class DriveCharacteristics {
    public static final double Ks = 1.268;
    public static final double Kv = 2.4389;
    public static final double Ka = 1.033586;
    public static final double Kp = 3.2843;
    public static final double Kd = 0.0;

    //TODO CHANGE THIS!
    public static final double trackWidthMeters = 0.5588;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(trackWidthMeters);

    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

    public static final int ENCODER_CPR = 4028;
    public static final double WHEEL_DIAMETER_METERS = 0.15;
    public static final double GEAR_RATIO = 8.45/1;
    public static final double ENCODER_GEAR_RATIO = 1;
    public static final double ENCODER_DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER_METERS / ENCODER_CPR / ENCODER_GEAR_RATIO / GEAR_RATIO * 1;

    public static final double ramseteB = 2;
    public static final double ramseteZeta = 0.7;
  }
  public static class PIDConstants {
    public static final double kp = 0.024;
    public static final double ki = 0.096;
    public static final double kd = 0.0025;

    public static final double Xkp = 0.3;
    public static final double Xki = 0.0;
    public static final double Xkd = 0.0;

    public static final double Rkp = 0.12;
    public static final double Rki = 0.11764;
    public static final double Rkd = 0.112;
  }
}

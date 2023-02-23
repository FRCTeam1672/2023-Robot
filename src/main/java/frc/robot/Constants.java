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
    public static final double Ks =  1.5089;
    public static final double Kv = 2.3322;
    public static final double Ka = 1.1693;
    public static final double Kp = 4.0742;
    public static final double Kd = 0.0;

    //TODO CHANGE THIS!
    public static final double trackWidthMeters = 0.69;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(trackWidthMeters);

    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

    public static final int ENCODER_CPR = 1024;
    public static final double WHEEL_DIAMETER_METERS = 0.15;
    public static final double ENCODER_DISTANCE_PER_PULSE =
            // Assumes the encoders are directly mounted on the wheel shafts
            (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

    public static final double ramseteB = 2;
    public static final double ramseteZeta = 0.7;
  }
}

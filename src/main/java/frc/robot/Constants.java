// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kMaxSpeed = 4.0; // Meters per second

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class LimelightConstants {
        // measurements in inches
        public static final int kPortHeight = 24;
        public static final int kLimelightHeight = 28;
        public static final int kLimelightAngle = 0;
        public static final int kLimelightOffset = 0;
    }

    public static final class DriveConstants {
        public static final double kMaxAngularSpeed = 2 * Math.PI; // One rotation per second

        public static final double kGearRatio = 10.71;
        // Meters (6 inch/2)
        public static final double kWheelRadius = 0.1524 / 2;
        public static final double kEncoderResolution = 2048;
        public static final double kDistancePerTick = (2 * Math.PI * kWheelRadius / kEncoderResolution) / kGearRatio;
        // Meters
        public static final double kTrackWidth = .6477 * 2;
    }

    public static final String DriveConstants = null;
}

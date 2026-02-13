package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoTune {

    // Odometry
    public static double TICKS_PER_REV = 8192.0;
    public static double WHEEL_DIAMETER = 2.0; // inches
    public static double INCHES_PER_TICK = 0.0; // 0 = compute from TICKS_PER_REV + WHEEL_DIAMETER

    // Drive PID
    public static double kP_TRANSLATION = 0.05;
    public static double kD_TRANSLATION = 0.0;
    public static double kP_HEADING = 0.02;
    public static double kD_HEADING = 0.0;

    // Go-to-pose targets
    public static double TARGET_X = 24.0;
    public static double TARGET_Y = 0.0;
    public static double TARGET_HEADING = 0.0; // degrees

    // Motion constraints
    public static double MAX_DRIVE_POWER = 0.6;
    public static double POSITION_TOLERANCE = 0.5; // inches
    public static double HEADING_TOLERANCE = 2.0;  // degrees

    // Pinpoint offsets (optional)
    public static double PINPOINT_X_OFFSET_MM = 0.0;
    public static double PINPOINT_Y_OFFSET_MM = 0.0;
}

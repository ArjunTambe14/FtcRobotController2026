package org.firstinspires.ftc.teamcode.util;

public class MathUtil {

    public static double clamp(double v, double min, double max){
        return Math.max(min, Math.min(max, v));
    }
}

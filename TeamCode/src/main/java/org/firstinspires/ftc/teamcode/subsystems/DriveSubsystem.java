package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class DriveSubsystem {

    RobotHardware r;

    public static double FAST = 1.0;
    public static double SLOW = 0.4;
    public static double DEAD = 0.05;

    public DriveSubsystem(RobotHardware robot) {
        r = robot;
    }

    public void update(Gamepad g) {

        double y = -dead(g.left_stick_y);
        double x = dead(g.left_stick_x);
        double rx = dead(g.right_stick_x);

        double mult = g.left_bumper ? SLOW : FAST;

        double fl = (y + x + rx);
        double bl = (y - x + rx);
        double fr = (y - x - rx);
        double br = (y + x - rx);

        double max = Math.max(1, Math.max(Math.abs(fl),
                Math.max(Math.abs(bl),
                        Math.max(Math.abs(fr), Math.abs(br)))));

        r.frontLeft.setPower(fl / max * mult);
        r.backLeft.setPower(bl / max * mult);
        r.frontRight.setPower(fr / max * mult);
        r.backRight.setPower(br / max * mult);
    }

    double dead(double v){
        return Math.abs(v) < DEAD ? 0 : v;
    }
}

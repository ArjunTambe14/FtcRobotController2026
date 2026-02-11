package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class DriveSubsystem {

    // Drive motor direction constants (flip FORWARD/REVERSE if a wheel spins wrong)
    public static DcMotorSimple.Direction FRONT_LEFT_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction FRONT_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction BACK_LEFT_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction BACK_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;

    private final RobotHardware r;
    private final double fastMult;
    private final double slowMult;
    private final double deadzone;
    private boolean slowMode = false;

    public DriveSubsystem(RobotHardware robot, double fastMult, double slowMult, double deadzone) {
        this.r = robot;
        this.fastMult = fastMult;
        this.slowMult = slowMult;
        this.deadzone = deadzone;

        r.frontLeft.setDirection(FRONT_LEFT_DIR);
        r.frontRight.setDirection(FRONT_RIGHT_DIR);
        r.backLeft.setDirection(BACK_LEFT_DIR);
        r.backRight.setDirection(BACK_RIGHT_DIR);

        r.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        r.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(Gamepad g) {

        double y = -applyDeadzone(g.left_stick_y);
        double x = applyDeadzone(g.left_stick_x);
        double rx = applyDeadzone(g.right_stick_x);

        slowMode = g.left_bumper;
        double mult = slowMode ? slowMult : fastMult;

        double fl = (y + x + rx);
        double bl = (y - x + rx);
        double fr = (y - x - rx);
        double br = (y + x - rx);

        double max = Math.max(1, Math.max(Math.abs(fl),
                Math.max(Math.abs(bl),
                        Math.max(Math.abs(fr), Math.abs(br)))));

        r.frontLeft.setPower(clip11((fl / max) * mult));
        r.backLeft.setPower(clip11((bl / max) * mult));
        r.frontRight.setPower(clip11((fr / max) * mult));
        r.backRight.setPower(clip11((br / max) * mult));
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    private double applyDeadzone(double v) {
        return Math.abs(v) < deadzone ? 0.0 : v;
    }

    private double clip11(double v) {
        return Range.clip(v, -1.0, 1.0);
    }
}

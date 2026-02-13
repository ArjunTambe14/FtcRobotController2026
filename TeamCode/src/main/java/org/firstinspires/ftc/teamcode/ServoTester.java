package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Tester", group = "Tools")
public class ServoTester extends OpMode {

    private static final double STICK_DEADZONE = 0.05;
    // Flip these if clockwise is reversed for a given servo
    private static final boolean BALL_PUSHER_INVERT = false;
    private static final boolean HOOD_SERVO_INVERT = false;
    private static final boolean SPINDEXER_INVERT = false;
    private static final boolean SUPPORT_WHEEL_INVERT = false;

    private Servo ballPusher;
    private Servo hoodServo;
    private Servo spindexerAxon;
    private CRServo supportWheelCR;

    private double ballPos = 0.5;
    private double hoodPos = 0.5;
    private double spinPos = 0.5;

    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);
        ballPusher = getServo("ballPusher");
        hoodServo = getServo("hoodServo");
        spindexerAxon = getServo("spindexerAxon");
        supportWheelCR = getCRServo("supportWheelCR");

        setServoIfPresent(ballPusher, ballPos);
        setServoIfPresent(hoodServo, hoodPos);
        setServoIfPresent(spindexerAxon, spinPos);

        telemetry.addLine("Servo Tester Initialized");
        updateTelemetry();
    }

    @Override
    public void init_loop() {
        updateTelemetry();
    }

    @Override
    public void loop() {
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        double stick = -gamepad1.left_stick_y;
        if (Math.abs(stick) < STICK_DEADZONE) {
            stick = 0.0;
        }
        double stickLX = gamepad1.left_stick_x;
        double stickRY = -gamepad1.right_stick_y;
        double stickRX = gamepad1.right_stick_x;

        if (Math.abs(stickLX) < STICK_DEADZONE) {
            stickLX = 0.0;
        }
        if (Math.abs(stickRY) < STICK_DEADZONE) {
            stickRY = 0.0;
        }
        if (Math.abs(stickRX) < STICK_DEADZONE) {
            stickRX = 0.0;
        }

        // Positional servos map -1..1 to 0..1
        ballPos = clip01((stick + 1.0) * 0.5);
        hoodPos = clip01((stickRY + 1.0) * 0.5);
        spinPos = clip01((stickLX + 1.0) * 0.5);

        if (BALL_PUSHER_INVERT) {
            ballPos = 1.0 - ballPos;
        }
        if (HOOD_SERVO_INVERT) {
            hoodPos = 1.0 - hoodPos;
        }
        if (SPINDEXER_INVERT) {
            spinPos = 1.0 - spinPos;
        }

        setServoIfPresent(ballPusher, ballPos);
        setServoIfPresent(hoodServo, hoodPos);
        setServoIfPresent(spindexerAxon, spinPos);

        // CR servo uses full -1..1 power
        if (supportWheelCR != null) {
            double power = Range.clip(stickRX, -1.0, 1.0);
            if (SUPPORT_WHEEL_INVERT) {
                power = -power;
            }
            supportWheelCR.setPower(power);
        }

        updateTelemetry();

        lastUp = up;
        lastDown = down;
    }

    private void updateTelemetry() {
        telemetry.addLine("Servo Tester (gamepad1)");
        telemetry.addLine("Left Y = ballPusher, Right Y = hoodServo");
        telemetry.addLine("Left X = spindexerAxon, Right X = supportWheelCR");
        telemetry.addLine("Flip *_INVERT flags if clockwise is reversed");
        telemetry.addData("ballPusher", fmtServo(ballPusher, ballPos));
        telemetry.addData("hoodServo", fmtServo(hoodServo, hoodPos));
        telemetry.addData("spindexerAxon", fmtServo(spindexerAxon, spinPos));
        telemetry.addData("supportWheelCR", supportWheelCR == null ? "MISSING" : "OK");
        telemetry.update();
    }

    private Servo getServo(String name) {
        try {
            return hardwareMap.get(Servo.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    private CRServo getCRServo(String name) {
        try {
            return hardwareMap.get(CRServo.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    private void setServoIfPresent(Servo servo, double pos) {
        if (servo != null) {
            servo.setPosition(clip01(pos));
        }
    }

    private String fmtServo(Servo servo, double pos) {
        if (servo == null) {
            return "MISSING";
        }
        return String.format("%.3f", pos);
    }

    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }
}

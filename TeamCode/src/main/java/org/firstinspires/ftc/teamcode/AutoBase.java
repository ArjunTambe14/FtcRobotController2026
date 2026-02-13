package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Auto Base", group = "Competition")
public class AutoBase extends OpMode {

    // ========================
    // EDIT THESE FIRST
    // ========================
    public static double DRIVE_FAST_MULT = 0.60;
    public static double STICK_DEADZONE = 0.05;

    // Basic auto steps (edit these to change the routine)
    private static final double STEP0_TIME = 1.00; // forward

    // Servo defaults
    public static double BALL_PUSHER_HOME = 0.50;
    public static double HOOD_START_POS = 0.50;
    public static double SPINDEXER_POS_0 = 0.20;

    // Drive motor direction constants (flip if needed)
    public static DcMotorSimple.Direction FRONT_LEFT_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction FRONT_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction BACK_LEFT_DIR = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction BACK_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;

    // ========================
    // Hardware
    // ========================
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private DcMotor shooterFlywheel;
    private DcMotor turretRotate;
    private Servo spindexerAxon;
    private CRServo supportWheelCR;
    private Servo ballPusher;
    private Servo hoodServo;

    private final ElapsedTime stepTimer = new ElapsedTime();
    private int stepIndex = 0;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        shooterFlywheel = hardwareMap.get(DcMotor.class, "shooterFlywheel");
        turretRotate = hardwareMap.get(DcMotor.class, "turretRotate");

        spindexerAxon = hardwareMap.get(Servo.class, "spindexerAxon");
        supportWheelCR = hardwareMap.get(CRServo.class, "supportWheelCR");
        ballPusher = hardwareMap.get(Servo.class, "ballPusher");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        frontLeft.setDirection(FRONT_LEFT_DIR);
        frontRight.setDirection(FRONT_RIGHT_DIR);
        backLeft.setDirection(BACK_LEFT_DIR);
        backRight.setDirection(BACK_RIGHT_DIR);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ballPusher.setPosition(clip01(BALL_PUSHER_HOME));
        hoodServo.setPosition(clip01(HOOD_START_POS));
        spindexerAxon.setPosition(clip01(SPINDEXER_POS_0));
        supportWheelCR.setPower(0.0);

        setDrivePower(0.0, 0.0, 0.0);
        stepIndex = 0;
        stepTimer.reset();

        telemetry.addLine("Auto Base Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (stepIndex) {
            case 0:
                setDrivePower(1.0 * DRIVE_FAST_MULT, 0.0, 0.0);
                if (stepTimer.seconds() >= STEP0_TIME) {
                    nextStep();
                }
                break;
            case 1:
                setDrivePower(0.0, 0.0, 0.0);
                break;
            default:
                setDrivePower(0.0, 0.0, 0.0);
                break;
        }

        telemetry.addData("Step", stepIndex);
        telemetry.addData("Step Time", String.format("%.2f", stepTimer.seconds()));
        telemetry.update();
    }

    private void nextStep() {
        stepIndex++;
        stepTimer.reset();
    }

    private void setDrivePower(double y, double x, double rx) {
        y = applyDeadzone(y, STICK_DEADZONE);
        x = applyDeadzone(x, STICK_DEADZONE);
        rx = applyDeadzone(rx, STICK_DEADZONE);

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(clip11(fl / max));
        backLeft.setPower(clip11(bl / max));
        frontRight.setPower(clip11(fr / max));
        backRight.setPower(clip11(br / max));
    }

    private double applyDeadzone(double v, double dz) {
        return Math.abs(v) < dz ? 0.0 : v;
    }

    private double clip11(double v) {
        return Range.clip(v, -1.0, 1.0);
    }

    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }

}

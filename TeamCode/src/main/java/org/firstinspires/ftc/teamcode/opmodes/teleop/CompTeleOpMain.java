package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.EdgeButton;

@TeleOp(name="Comp Tele Main", group="Competition")
public class CompTeleOpMain extends OpMode {

    // ========================
    // EDIT THESE FIRST
    // ========================
    public static double DRIVE_FAST_MULT = 1.00;
    public static double DRIVE_SLOW_MULT = 0.40;
    public static double STICK_DEADZONE = 0.05;
    public static double TRIGGER_DEADZONE = 0.10;

    // Intake tuning
    public static double INTAKE_TARGET_POWER_DEFAULT = 0.70;
    public static double INTAKE_STEP = 0.05;

    // Flywheel tuning
    public static double FLYWHEEL_TARGET_POWER_DEFAULT = 0.80;
    public static double FLYWHEEL_STEP = 0.05;

    // Support wheel direction (FEED must match CAD direction)
    public static double SUPPORT_WHEEL_POWER_FEED = -1.0; // flip sign if direction is reversed

    // Ball pusher tuning (placeholders, easy to adjust)
    public static double BALL_PUSHER_HOME = 0.50;
    public static double BALL_PUSHER_PUSH = 0.80;
    public static double BALL_PUSHER_HOLD_TIME_SEC = 0.25;

    // Hood tuning
    public static double HOOD_START_POS = 0.50;
    public static double HOOD_ADJUST_RATE = 0.01;

    // Spindexer 3-slot positions (placeholders, easy to adjust)
    public static double SPINDEXER_POS_0 = 0.20;
    public static double SPINDEXER_POS_1 = 0.50;
    public static double SPINDEXER_POS_2 = 0.80;

    // Turret anti-tangle constants
    public static double MOTOR_ENCODER_PPR = 28.0;
    public static double QUADRATURE_MULT = 4.0;
    public static double TURRET_MOTOR_GEAR_RATIO = 13.7; // 435rpm gearbox default; keep editable
    public static double PINION_TEETH = 80.0;
    public static double TURRET_GEAR_TEETH = 170.0;
    public static double TURRET_LIMIT_DEG = 150.0;
    public static double TURRET_UNWIND_MARGIN_DEG = 10.0;
    public static double TURRET_UNWIND_POWER = 0.60;
    public static double TURRET_DRIVER_MAX_POWER = 0.50;
    public static double TURRET_DEADZONE = 0.05;

    // Limelight AprilTag yaw tracking (auto-aim)
    public static double LIMELIGHT_YAW_KP = 0.02;
    public static double LIMELIGHT_YAW_DEADZONE_DEG = 1.0;
    public static double LIMELIGHT_MAX_POWER = 0.35;
    public static double LIMELIGHT_YAW_SIGN = 1.0; // flip if auto-aim turns the wrong way
    public static int LIMELIGHT_PIPELINE = 0;

    private RobotHardware robot;

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private SpindexerSubsystem spindexer;

    private final EdgeButton tuneLeft = new EdgeButton();
    private final EdgeButton tuneRight = new EdgeButton();
    private final EdgeButton tuneUp = new EdgeButton();
    private final EdgeButton tuneDown = new EdgeButton();
    private int tuningIndex = 0;
    private Limelight3A limelight;
    private boolean limelightAvailable = false;
    private boolean limelightHasTarget = false;
    private double limelightYawDeg = 0.0;

    private static final TuningParam[] TUNING_ORDER = new TuningParam[]{
            TuningParam.BALL_HOME,
            TuningParam.BALL_PUSH,
            TuningParam.HOOD_POS,
            TuningParam.SPIN_POS0,
            TuningParam.SPIN_POS1,
            TuningParam.SPIN_POS2,
            TuningParam.SUPPORT_SIGN
    };

    // PlayStation mapping reminder:
    // Square = gamepad2.x, Circle = gamepad2.b, Cross = gamepad2.a, Triangle = gamepad2.y

    @Override
    public void init() {

        robot = new RobotHardware();
        robot.init(hardwareMap);

        drive = new DriveSubsystem(robot, DRIVE_FAST_MULT, DRIVE_SLOW_MULT, STICK_DEADZONE);
        intake = new IntakeSubsystem(robot, INTAKE_TARGET_POWER_DEFAULT, INTAKE_STEP, TRIGGER_DEADZONE);
        spindexer = new SpindexerSubsystem(robot, SPINDEXER_POS_0, SPINDEXER_POS_1, SPINDEXER_POS_2);
        shooter = new ShooterSubsystem(robot, spindexer,
                FLYWHEEL_TARGET_POWER_DEFAULT, FLYWHEEL_STEP, TRIGGER_DEADZONE,
                BALL_PUSHER_HOME, BALL_PUSHER_PUSH, BALL_PUSHER_HOLD_TIME_SEC,
                HOOD_START_POS, HOOD_ADJUST_RATE, SUPPORT_WHEEL_POWER_FEED);
        turret = new TurretSubsystem(robot);
        turret.configure(MOTOR_ENCODER_PPR, QUADRATURE_MULT, TURRET_MOTOR_GEAR_RATIO,
                PINION_TEETH, TURRET_GEAR_TEETH, TURRET_LIMIT_DEG, TURRET_UNWIND_MARGIN_DEG,
                TURRET_UNWIND_POWER, TURRET_DRIVER_MAX_POWER, TURRET_DEADZONE);
        turret.configureAuto(LIMELIGHT_YAW_KP, LIMELIGHT_YAW_DEADZONE_DEG, LIMELIGHT_MAX_POWER, LIMELIGHT_YAW_SIGN);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();
            limelightAvailable = true;
        } catch (Exception e) {
            limelightAvailable = false;
        }

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        boolean tuningMode = gamepad2.y; // hold triangle for tuning
        boolean tuneLeftPressed = tuneLeft.rising(gamepad2.dpad_left);
        boolean tuneRightPressed = tuneRight.rising(gamepad2.dpad_right);
        boolean tuneUpPressed = tuneUp.rising(gamepad2.dpad_up);
        boolean tuneDownPressed = tuneDown.rising(gamepad2.dpad_down);
        boolean autoAimEnabled = !tuningMode && gamepad2.right_bumper; // hold to auto-aim

        if (tuningMode) {
            if (tuneRightPressed) {
                tuningIndex = (tuningIndex + 1) % TUNING_ORDER.length;
            }
            if (tuneLeftPressed) {
                tuningIndex = (tuningIndex - 1 + TUNING_ORDER.length) % TUNING_ORDER.length;
            }

            double delta = 0.0;
            if (tuneUpPressed) {
                delta = 0.01;
            } else if (tuneDownPressed) {
                delta = -0.01;
            }

            switch (TUNING_ORDER[tuningIndex]) {
                case BALL_HOME:
                    if (delta != 0.0) {
                        shooter.setBallPusherHome(shooter.getBallPusherHome() + delta);
                    }
                    break;
                case BALL_PUSH:
                    if (delta != 0.0) {
                        shooter.setBallPusherPush(shooter.getBallPusherPush() + delta);
                    }
                    break;
                case HOOD_POS:
                    if (delta != 0.0) {
                        shooter.setHoodPos(shooter.getHoodPos() + delta);
                    }
                    break;
                case SPIN_POS0:
                    if (delta != 0.0) {
                        spindexer.setPositions(spindexer.getPos0() + delta, spindexer.getPos1(), spindexer.getPos2());
                    }
                    break;
                case SPIN_POS1:
                    if (delta != 0.0) {
                        spindexer.setPositions(spindexer.getPos0(), spindexer.getPos1() + delta, spindexer.getPos2());
                    }
                    break;
                case SPIN_POS2:
                    if (delta != 0.0) {
                        spindexer.setPositions(spindexer.getPos0(), spindexer.getPos1(), spindexer.getPos2() + delta);
                    }
                    break;
                case SUPPORT_SIGN:
                    if (tuneUpPressed || tuneDownPressed) {
                        double sign = shooter.getSupportWheelFeed() >= 0.0 ? -1.0 : 1.0;
                        shooter.setSupportWheelFeed(sign);
                    }
                    break;
                default:
                    break;
            }
        }

        if (tuningMode) {
            TuningParam param = TUNING_ORDER[tuningIndex];
            if (param == TuningParam.BALL_HOME) {
                shooter.setBallPusherPreview(shooter.getBallPusherHome());
            } else if (param == TuningParam.BALL_PUSH) {
                shooter.setBallPusherPreview(shooter.getBallPusherPush());
            } else {
                shooter.setBallPusherPreview(null);
            }

            if (param == TuningParam.SPIN_POS0) {
                spindexer.setOverridePosition(spindexer.getPos0());
            } else if (param == TuningParam.SPIN_POS1) {
                spindexer.setOverridePosition(spindexer.getPos1());
            } else if (param == TuningParam.SPIN_POS2) {
                spindexer.setOverridePosition(spindexer.getPos2());
            } else {
                spindexer.clearOverride();
            }
        } else {
            shooter.setBallPusherPreview(null);
            spindexer.clearOverride();
        }

        drive.update(gamepad1);

        intake.update(gamepad1);

        if (limelightAvailable) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                limelightYawDeg = result.getTx();
                limelightHasTarget = !result.getFiducialResults().isEmpty();
            } else {
                limelightYawDeg = 0.0;
                limelightHasTarget = false;
            }
        } else {
            limelightYawDeg = 0.0;
            limelightHasTarget = false;
        }

        turret.setAutoTracking(autoAimEnabled, limelightHasTarget, limelightYawDeg);

        shooter.update(gamepad2, tuningMode);

        spindexer.update(gamepad2, tuningMode);

        turret.update(gamepad2);

        if (tuningMode) {
            telemetry.addLine("TUNING MODE ON");
            telemetry.addData("Param", TUNING_ORDER[tuningIndex]);
            telemetry.addData("Value", String.format("%.3f", getTuningValue(TUNING_ORDER[tuningIndex])));
            telemetry.addLine("DPAD L/R select, UP/DOWN adjust");
            telemetry.addData("Support Sign", shooter.getSupportWheelFeed() >= 0.0 ? "+1" : "-1");
        }

        telemetry.addData("Intake Target", String.format("%.2f", intake.getIntakeTargetPower()));
        telemetry.addData("Flywheel Target", String.format("%.2f", shooter.getFlywheelTargetPower()));
        telemetry.addData("Hood Pos", String.format("%.3f", shooter.getHoodPos()));
        telemetry.addData("Spindexer Step", spindexer.getStep());
        telemetry.addData("Spindexer Pos", String.format("%.3f", spindexer.getStepPosition()));
        telemetry.addData("Shot State", shooter.getShotState());
        telemetry.addData("Shot Timer", String.format("%.2f", shooter.getShotTime()));
        telemetry.addData("SupportWheel", String.format("%.2f", shooter.getSupportWheelPower()));
        telemetry.addData("Turret Deg", String.format("%.1f", turret.getTurretDegrees()));
        telemetry.addData("Turret Unwind", turret.isUnwinding() ? "ON" : "OFF");
        telemetry.addData("Turret Auto", turret.isAutoEnabled() ? "ON" : "OFF");
        telemetry.addData("LL Avail", limelightAvailable ? "YES" : "NO");
        telemetry.addData("LL HasTarget", limelightHasTarget ? "YES" : "NO");
        telemetry.addData("LL Yaw", String.format("%.2f", limelightYawDeg));
        telemetry.addData("LL AutoCmd", String.format("%.2f", turret.getAutoCommand()));
        telemetry.addData("Slow Mode", drive.isSlowMode() ? "ON" : "OFF");

        telemetry.addLine("---- COPY CONSTANTS ----");
        telemetry.addLine(String.format("BALL_PUSHER_HOME = %.3f", shooter.getBallPusherHome()));
        telemetry.addLine(String.format("BALL_PUSHER_PUSH = %.3f", shooter.getBallPusherPush()));
        telemetry.addLine(String.format("HOOD_START_POS = %.3f", shooter.getHoodPos()));
        telemetry.addLine(String.format("SPINDEXER_POS_0 = %.3f", spindexer.getPos0()));
        telemetry.addLine(String.format("SPINDEXER_POS_1 = %.3f", spindexer.getPos1()));
        telemetry.addLine(String.format("SPINDEXER_POS_2 = %.3f", spindexer.getPos2()));
        telemetry.addLine(String.format("SUPPORT_WHEEL_POWER_FEED = %.3f", shooter.getSupportWheelFeed()));
        telemetry.update();
    }

    private double getTuningValue(TuningParam param) {
        switch (param) {
            case BALL_HOME:
                return shooter.getBallPusherHome();
            case BALL_PUSH:
                return shooter.getBallPusherPush();
            case HOOD_POS:
                return shooter.getHoodPos();
            case SPIN_POS0:
                return spindexer.getPos0();
            case SPIN_POS1:
                return spindexer.getPos1();
            case SPIN_POS2:
                return spindexer.getPos2();
            case SUPPORT_SIGN:
                return shooter.getSupportWheelFeed();
            default:
                return 0.0;
        }
    }

    @Override
    public void stop() {
        if (limelightAvailable && limelight != null) {
            limelight.stop();
        }
    }

    private enum TuningParam {
        BALL_HOME,
        BALL_PUSH,
        HOOD_POS,
        SPIN_POS0,
        SPIN_POS1,
        SPIN_POS2,
        SUPPORT_SIGN
    }
}

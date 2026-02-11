package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.EdgeButton;

public class ShooterSubsystem {

    private final RobotHardware r;
    private final SpindexerSubsystem spindexer;
    private final EdgeButton dpadUp = new EdgeButton();
    private final EdgeButton dpadDown = new EdgeButton();

    private double flywheelTargetPower;
    private final double flywheelStep;
    private final double triggerDeadzone;
    private double supportWheelFeed;
    private double ballPusherHome;
    private double ballPusherPush;
    private final double ballPusherHoldTimeSec;
    private double hoodPos;
    private final double hoodAdjustRate;

    private ShotState state = ShotState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean prevRightTrigger = false;
    private double lastSupportWheelPower = 0.0;
    private double lastFlywheelPower = 0.0;
    private Double ballPusherPreview = null;

    public ShooterSubsystem(RobotHardware robot, SpindexerSubsystem spin,
                            double flywheelTargetPowerDefault, double flywheelStep, double triggerDeadzone,
                            double ballPusherHome, double ballPusherPush, double ballPusherHoldTimeSec,
                            double hoodStartPos, double hoodAdjustRate, double supportWheelFeed) {
        r = robot;
        spindexer = spin;
        this.flywheelTargetPower = flywheelTargetPowerDefault;
        this.flywheelStep = flywheelStep;
        this.triggerDeadzone = triggerDeadzone;
        this.ballPusherHome = ballPusherHome;
        this.ballPusherPush = ballPusherPush;
        this.ballPusherHoldTimeSec = ballPusherHoldTimeSec;
        this.hoodPos = hoodStartPos;
        this.hoodAdjustRate = hoodAdjustRate;
        this.supportWheelFeed = supportWheelFeed;

        r.shooterFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.ballPusher.setPosition(clip01(ballPusherHome));
        r.hoodServo.setPosition(clip01(hoodPos));
        r.supportWheelCR.setPower(0.0);
    }

    public void update(Gamepad g, boolean tuningMode) {
        boolean readyActive = !tuningMode && (g.left_trigger > triggerDeadzone);

        boolean rightTriggerActive = g.right_trigger > triggerDeadzone;
        boolean rightTriggerPressed = rightTriggerActive && !prevRightTrigger;
        prevRightTrigger = rightTriggerActive;

        boolean dpadUpPressed = dpadUp.rising(g.dpad_up);
        boolean dpadDownPressed = dpadDown.rising(g.dpad_down);

        if (!tuningMode) {
            if (dpadUpPressed) {
                flywheelTargetPower = clip01(flywheelTargetPower + flywheelStep);
            }
            if (dpadDownPressed) {
                flywheelTargetPower = clip01(flywheelTargetPower - flywheelStep);
            }

            hoodPos = clip01(hoodPos + (-g.right_stick_y) * hoodAdjustRate);
        }

        if (rightTriggerPressed && !tuningMode && state == ShotState.IDLE) {
            state = ShotState.PUSHING;
            timer.reset();
        }

        switch (state) {
            case PUSHING:
                state = ShotState.HOLDING;
                break;
            case HOLDING:
                if (timer.seconds() >= ballPusherHoldTimeSec) {
                    spindexer.stepForward();
                    state = ShotState.RETURNING;
                }
                break;
            case RETURNING:
                state = ShotState.IDLE;
                timer.reset();
                break;
            case IDLE:
            default:
                break;
        }

        boolean shotActive = state != ShotState.IDLE;

        double flywheelPower = 0.0;
        double supportWheelPower = 0.0;
        if (readyActive || shotActive) {
            flywheelPower = flywheelTargetPower;
            supportWheelPower = supportWheelFeed;
        }
        lastFlywheelPower = flywheelPower;
        lastSupportWheelPower = supportWheelPower;
        r.shooterFlywheel.setPower(clip11(flywheelPower));
        r.supportWheelCR.setPower(clip11(supportWheelPower));

        // Hood servo always tracks hoodPos
        r.hoodServo.setPosition(clip01(hoodPos));

        // Ball pusher control
        if (shotActive) {
            if (state == ShotState.HOLDING) {
                r.ballPusher.setPosition(clip01(ballPusherPush));
            } else {
                r.ballPusher.setPosition(clip01(ballPusherHome));
            }
        } else if (ballPusherPreview != null) {
            r.ballPusher.setPosition(clip01(ballPusherPreview));
        } else {
            r.ballPusher.setPosition(clip01(ballPusherHome));
        }
    }

    public double getFlywheelTargetPower() {
        return flywheelTargetPower;
    }

    public double getHoodPos() {
        return hoodPos;
    }

    public ShotState getShotState() {
        return state;
    }

    public double getShotTime() {
        return timer.seconds();
    }

    public double getSupportWheelPower() {
        return lastSupportWheelPower;
    }

    public double getFlywheelPower() {
        return lastFlywheelPower;
    }

    public double getBallPusherHome() {
        return ballPusherHome;
    }

    public double getBallPusherPush() {
        return ballPusherPush;
    }

    public void setBallPusherHome(double pos) {
        ballPusherHome = clip01(pos);
    }

    public void setBallPusherPush(double pos) {
        ballPusherPush = clip01(pos);
    }

    public void setBallPusherPreview(Double pos) {
        ballPusherPreview = (pos == null) ? null : clip01(pos);
    }

    public void setHoodPos(double pos) {
        hoodPos = clip01(pos);
    }

    public void setSupportWheelFeed(double power) {
        supportWheelFeed = clip11(power);
    }

    public double getSupportWheelFeed() {
        return supportWheelFeed;
    }

    public double getFlywheelStep() {
        return flywheelStep;
    }

    private double clip11(double v) {
        return Range.clip(v, -1.0, 1.0);
    }

    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }

    public enum ShotState {
        IDLE,
        PUSHING,
        HOLDING,
        RETURNING
    }
}

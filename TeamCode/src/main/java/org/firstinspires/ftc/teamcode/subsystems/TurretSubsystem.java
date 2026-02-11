package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class TurretSubsystem {

    // Turret direction (flip if joystick right does not rotate turret clockwise)
    public static DcMotorSimple.Direction TURRET_DIR = DcMotorSimple.Direction.FORWARD;

    private final RobotHardware r;

    private double motorEncoderPPR = 28.0;
    private double quadratureMult = 4.0;
    private double motorGearRatio = 13.7;
    private double pinionTeeth = 80.0;
    private double turretGearTeeth = 170.0;
    private double limitDeg = 150.0;
    private double unwindMarginDeg = 10.0;
    private double unwindPower = 0.60;
    private double driverMaxPower = 0.50;
    private double deadzone = 0.05;
    private double autoKp = 0.02;
    private double autoDeadzoneDeg = 1.0;
    private double autoMaxPower = 0.35;
    private double autoYawSign = 1.0;

    private boolean autoEnabled = false;
    private boolean autoHasTarget = false;
    private double autoYawDeg = 0.0;
    private double autoCommand = 0.0;

    private double turretDegrees = 0.0;
    private int unwindDir = 0; // -1 unwind negative, +1 unwind positive, 0 none

    public TurretSubsystem(RobotHardware robot) {
        r = robot;

        r.turretRotate.setDirection(TURRET_DIR);
        r.turretRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.turretRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // only reset once in init
        r.turretRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void configure(double motorEncoderPPR, double quadratureMult, double motorGearRatio,
                          double pinionTeeth, double turretGearTeeth, double limitDeg,
                          double unwindMarginDeg, double unwindPower, double driverMaxPower,
                          double deadzone) {
        this.motorEncoderPPR = motorEncoderPPR;
        this.quadratureMult = quadratureMult;
        this.motorGearRatio = motorGearRatio;
        this.pinionTeeth = pinionTeeth;
        this.turretGearTeeth = turretGearTeeth;
        this.limitDeg = limitDeg;
        this.unwindMarginDeg = unwindMarginDeg;
        this.unwindPower = unwindPower;
        this.driverMaxPower = driverMaxPower;
        this.deadzone = deadzone;
    }

    public void configureAuto(double autoKp, double autoDeadzoneDeg, double autoMaxPower, double autoYawSign) {
        this.autoKp = autoKp;
        this.autoDeadzoneDeg = autoDeadzoneDeg;
        this.autoMaxPower = autoMaxPower;
        this.autoYawSign = autoYawSign;
    }

    public void setAutoTracking(boolean enabled, boolean hasTarget, double yawDeg) {
        this.autoEnabled = enabled;
        this.autoHasTarget = hasTarget;
        this.autoYawDeg = yawDeg;
    }

    public void update(Gamepad g) {
        turretDegrees = computeTurretDegrees();

        double cmd = applyDeadzone(g.left_stick_x, deadzone) * driverMaxPower;
        boolean driverActive = Math.abs(cmd) > 0.0;

        double autoCmd = 0.0;
        if (autoEnabled && autoHasTarget) {
            double error = autoYawDeg * autoYawSign;
            if (Math.abs(error) > autoDeadzoneDeg) {
                autoCmd = clip11(error * autoKp);
                autoCmd = Range.clip(autoCmd, -autoMaxPower, autoMaxPower);
            }
        }
        autoCommand = autoCmd;

        if (autoEnabled && autoHasTarget && !driverActive) {
            cmd = autoCmd;
        }

        if (unwindDir == 0) {
            if (turretDegrees >= limitDeg && cmd > 0) {
                unwindDir = -1;
            } else if (turretDegrees <= -limitDeg && cmd < 0) {
                unwindDir = 1;
            }
        } else if (unwindDir < 0) {
            if (turretDegrees <= (limitDeg - unwindMarginDeg)) {
                unwindDir = 0;
            }
        } else if (unwindDir > 0) {
            if (turretDegrees >= (-limitDeg + unwindMarginDeg)) {
                unwindDir = 0;
            }
        }

        boolean unwindActive = unwindDir != 0;
        double power = unwindActive ? (unwindDir * unwindPower) : cmd;

        // If driver is commanding back toward center, allow their control.
        if ((turretDegrees >= limitDeg && cmd < 0) || (turretDegrees <= -limitDeg && cmd > 0)) {
            power = cmd;
        }

        r.turretRotate.setPower(clip11(power));
    }

    public double getTurretDegrees() {
        return turretDegrees;
    }

    public boolean isUnwinding() {
        return unwindDir != 0;
    }

    public boolean isAutoEnabled() {
        return autoEnabled;
    }

    public boolean hasAutoTarget() {
        return autoHasTarget;
    }

    public double getAutoYawDeg() {
        return autoYawDeg;
    }

    public double getAutoCommand() {
        return autoCommand;
    }

    private double computeTurretDegrees() {
        double ticks = r.turretRotate.getCurrentPosition();
        double motorOutputCountsPerRev = motorEncoderPPR * quadratureMult * motorGearRatio;
        double motorOutputRevs = ticks / motorOutputCountsPerRev;
        double turretRevs = motorOutputRevs * (pinionTeeth / turretGearTeeth);
        return turretRevs * 360.0;
    }

    private double applyDeadzone(double v, double dz) {
        return Math.abs(v) < dz ? 0.0 : v;
    }

    private double clip11(double v) {
        return Range.clip(v, -1.0, 1.0);
    }
}

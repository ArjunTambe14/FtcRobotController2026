//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.canvas.Canvas;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.util.AutoTune;
//
//@TeleOp(name = "Auto Dashboard Tune", group = "Tools")
//public class AutoDashboardTune extends OpMode {
//
//    private DcMotor frontLeft;
//    private DcMotor frontRight;
//    private DcMotor backLeft;
//    private DcMotor backRight;
//
//    private GoBildaPinpointDriver pinpoint;
//
//    private final ElapsedTime loopTimer = new ElapsedTime();
//
//    private double lastErrorX = 0.0;
//    private double lastErrorY = 0.0;
//    private double lastHeadingError = 0.0;
//    private double lastInchesPerTick = -1.0;
//
//    private double lastForward = 0.0;
//    private double lastStrafe = 0.0;
//    private double lastTurn = 0.0;
//    private double lastFl = 0.0;
//    private double lastFr = 0.0;
//    private double lastBl = 0.0;
//    private double lastBr = 0.0;
//
//    @Override
//    public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//        frontLeft.setDirection(DriveSubsystem.FRONT_LEFT_DIR);
//        frontRight.setDirection(DriveSubsystem.FRONT_RIGHT_DIR);
//        backLeft.setDirection(DriveSubsystem.BACK_LEFT_DIR);
//        backRight.setDirection(DriveSubsystem.BACK_RIGHT_DIR);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        configurePinpoint();
//
//        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
//
//        loopTimer.reset();
//        telemetry.addLine("Auto Dashboard Tune Initialized");
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        double dt = Math.max(1e-3, loopTimer.seconds());
//        loopTimer.reset();
//
//        updatePinpointResolution();
//        pinpoint.update();
//        Pose2D pose = pinpoint.getPosition();
//
//        double x = pose.getX(DistanceUnit.INCH);
//        double y = pose.getY(DistanceUnit.INCH);
//        double heading = pose.getHeading(AngleUnit.DEGREES);
//
//        double errorX = AutoTune.TARGET_X - x;
//        double errorY = AutoTune.TARGET_Y - y;
//        double headingError = normalizeDegrees(AutoTune.TARGET_HEADING - heading);
//        double translationalError = Math.hypot(errorX, errorY);
//
//        double vx = AutoTune.kP_TRANSLATION * errorX + AutoTune.kD_TRANSLATION * ((errorX - lastErrorX) / dt);
//        double vy = AutoTune.kP_TRANSLATION * errorY + AutoTune.kD_TRANSLATION * ((errorY - lastErrorY) / dt);
//        double omega = AutoTune.kP_HEADING * headingError + AutoTune.kD_HEADING * ((headingError - lastHeadingError) / dt);
//
//        if (translationalError < AutoTune.POSITION_TOLERANCE) {
//            vx = 0.0;
//            vy = 0.0;
//        }
//        if (Math.abs(headingError) < AutoTune.HEADING_TOLERANCE) {
//            omega = 0.0;
//        }
//
//        vx = Range.clip(vx, -AutoTune.MAX_DRIVE_POWER, AutoTune.MAX_DRIVE_POWER);
//        vy = Range.clip(vy, -AutoTune.MAX_DRIVE_POWER, AutoTune.MAX_DRIVE_POWER);
//        omega = Range.clip(omega, -AutoTune.MAX_DRIVE_POWER, AutoTune.MAX_DRIVE_POWER);
//
//        double headingRad = Math.toRadians(heading);
//        double robotX = vx * Math.cos(headingRad) + vy * Math.sin(headingRad);
//        double robotY = -vx * Math.sin(headingRad) + vy * Math.cos(headingRad);
//
//        setDrivePower(robotY, robotX, omega, AutoTune.MAX_DRIVE_POWER);
//
//        lastErrorX = errorX;
//        lastErrorY = errorY;
//        lastHeadingError = headingError;
//        lastForward = robotY;
//        lastStrafe = robotX;
//        lastTurn = omega;
//
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("x", x);
//        packet.put("y", y);
//        packet.put("heading", heading);
//        packet.put("translationalError", translationalError);
//        packet.put("headingError", headingError);
//        packet.put("forwardCmd", lastForward);
//        packet.put("strafeCmd", lastStrafe);
//        packet.put("turnCmd", lastTurn);
//        packet.put("fl", lastFl);
//        packet.put("fr", lastFr);
//        packet.put("bl", lastBl);
//        packet.put("br", lastBr);
//        drawRobot(packet.fieldOverlay(), x, y, headingRad);
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//
//        telemetry.addData("x", String.format("%.2f", x));
//        telemetry.addData("y", String.format("%.2f", y));
//        telemetry.addData("heading", String.format("%.1f", heading));
//        telemetry.addData("translationalError", String.format("%.2f", translationalError));
//        telemetry.addData("headingError", String.format("%.1f", headingError));
//        telemetry.addData("forwardCmd", String.format("%.2f", lastForward));
//        telemetry.addData("strafeCmd", String.format("%.2f", lastStrafe));
//        telemetry.addData("turnCmd", String.format("%.2f", lastTurn));
//        telemetry.addData("fl", String.format("%.2f", lastFl));
//        telemetry.addData("fr", String.format("%.2f", lastFr));
//        telemetry.addData("bl", String.format("%.2f", lastBl));
//        telemetry.addData("br", String.format("%.2f", lastBr));
//        telemetry.update();
//    }
//
//    private void configurePinpoint() {
//        pinpoint.setOffsets(AutoTune.PINPOINT_X_OFFSET_MM, AutoTune.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
//        updatePinpointResolution();
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        pinpoint.resetPosAndIMU();
//    }
//
//    private void updatePinpointResolution() {
//        double inchesPerTick = AutoTune.INCHES_PER_TICK;
//        if (inchesPerTick <= 0.0) {
//            inchesPerTick = (Math.PI * AutoTune.WHEEL_DIAMETER) / AutoTune.TICKS_PER_REV;
//        }
//        if (Math.abs(inchesPerTick - lastInchesPerTick) > 1e-6) {
//            double ticksPerInch = 1.0 / inchesPerTick;
//            double ticksPerMm = ticksPerInch / 25.4;
//            pinpoint.setEncoderResolution(ticksPerMm, DistanceUnit.MM);
//            lastInchesPerTick = inchesPerTick;
//        }
//    }
//
//    private void setDrivePower(double y, double x, double rx, double maxPower) {
//        double fl = y + x + rx;
//        double bl = y - x + rx;
//        double fr = y - x - rx;
//        double br = y + x - rx;
//
//        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
//                Math.max(Math.abs(fr), Math.abs(br)))));
//
//        fl = (fl / max) * maxPower;
//        bl = (bl / max) * maxPower;
//        fr = (fr / max) * maxPower;
//        br = (br / max) * maxPower;
//
//        frontLeft.setPower(fl);
//        backLeft.setPower(bl);
//        frontRight.setPower(fr);
//        backRight.setPower(br);
//
//        lastFl = fl;
//        lastBl = bl;
//        lastFr = fr;
//        lastBr = br;
//    }
//
//    private void drawRobot(Canvas canvas, double x, double y, double headingRad) {
//        double r = 9.0;
//        double hx = x + r * Math.cos(headingRad);
//        double hy = y + r * Math.sin(headingRad);
//        canvas.strokeCircle(x, y, r);
//        canvas.strokeLine(x, y, hx, hy);
//    }
//
//    private double normalizeDegrees(double angle) {
//        while (angle > 180.0) {
//            angle -= 360.0;
//        }
//        while (angle < -180.0) {
//            angle += 360.0;
//        }
//        return angle;
//    }
//}

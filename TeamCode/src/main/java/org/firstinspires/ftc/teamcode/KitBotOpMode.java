package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.KitTurretControl;
import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;
import org.firstinspires.ftc.teamcode.vision.Limelight;
import org.firstinspires.ftc.teamcode.vision.TurretAutoAim;

@TeleOp(name ="KitBotOpModeDecode")
public class KitBotOpMode extends OpMode {

    MechanumDrive drive = new MechanumDrive();
    KitTurretControl turret = new KitTurretControl();
    Limelight limelight;
    TurretAutoAim autoAim;

    // auto-aim proportional constant
    private final double kP_pitch = 0.01;
    double forward, strafe, rotate;

    @Override
    public void init() {
        drive.init(hardwareMap);
        turret.init(hardwareMap);

        limelight = new Limelight("10.0.0.11"); // <-- replace with Limelight IP
        autoAim = new TurretAutoAim(turret, limelight, kP_pitch, 20);//edit tag here

        telemetry.addLine("✅ KitBot Initialized — ready to run");
        telemetry.update();
    }

    @Override
    public void loop() {
        //drive
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.driveFeildRelative(forward, strafe, rotate);
        //turret shoot
        double leftTrigger  = gamepad2.left_trigger;
        double rightTrigger = gamepad2.right_trigger;
        turret.runWithTriggers(leftTrigger, rightTrigger);

        //Autoaim
        boolean autoAlign = gamepad2.square;// auto-align toggle
        double manualPitch = gamepad2.right_stick_y;
        autoAim.update(autoAlign, manualPitch);

        // telemetry
        telemetry.addData("Drive", "F: %.2f  S: %.2f  R: %.2f", forward, strafe, rotate);
        telemetry.addData("Turret", "LeftTrig: %.2f  RightTrig: %.2f  StickY: %.2f", leftTrigger, rightTrigger, manualPitch);
        telemetry.addData("AutoAlign", autoAlign);
        telemetry.addData("Target ID", limelight.getTargetID());
        telemetry.addData("Limelight TY", limelight.getTY());
        telemetry.update();

    }
}

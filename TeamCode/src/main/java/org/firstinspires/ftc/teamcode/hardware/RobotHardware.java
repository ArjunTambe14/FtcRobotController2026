package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.*;

public class RobotHardware {

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor intakeLeft, intakeRight;
    public DcMotor shooterFlywheel;
    public DcMotor turretRotate;

    public Servo spindexerAxon;
    public CRServo supportWheelCR;
    public Servo ballPusher;
    public Servo hoodServo;

    public void init(HardwareMap hw) {

        frontLeft = hw.get(DcMotor.class, "frontLeft");
        frontRight = hw.get(DcMotor.class, "frontRight");
        backLeft = hw.get(DcMotor.class, "backLeft");
        backRight = hw.get(DcMotor.class, "backRight");

        intakeLeft = hw.get(DcMotor.class, "intakeLeft");
        intakeRight = hw.get(DcMotor.class, "intakeRight");

        shooterFlywheel = hw.get(DcMotor.class, "shooterFlywheel");
        turretRotate = hw.get(DcMotor.class, "turretRotate");

        spindexerAxon = hw.get(Servo.class, "spindexerAxon");
        supportWheelCR = hw.get(CRServo.class, "supportWheelCR");
        ballPusher = hw.get(Servo.class, "ballPusher");
        hoodServo = hw.get(Servo.class, "hoodServo");
    }
}

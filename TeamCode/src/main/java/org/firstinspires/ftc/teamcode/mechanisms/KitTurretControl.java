package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;

public class KitTurretControl {

    private DcMotor leftRowMotor, rightRowMotor;
    private Servo leftPitchServo, rightPitchServo;
    private double pitchPosition = 0.5; //servo position
    private final double PITCH_SPEED = 0.01; //sensitivity, change from code

    public void stop() {
        rightRowMotor.setPower(0);
        leftRowMotor.setPower(0);
    }

    private void setPitchPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));
        leftPitchServo.setPosition(position);
        rightPitchServo.setPosition(1.0 - position);
    }

    public void init(HardwareMap hwMap){

        leftRowMotor = hwMap.get(DcMotor.class, "leftRowMotor");
        rightRowMotor = hwMap.get(DcMotor.class, "rightRowMotor");

        rightRowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPitchServo = hwMap.get(Servo.class, "leftPitchServo");
        rightPitchServo = hwMap.get(Servo.class, "rightPitchServo");
        setPitchPosition(pitchPosition);

        stop();
    }

    public void runWithTriggers(double leftTrigger, double rightTrigger) {
        double power = 0;
        if (leftTrigger > 0.05 && rightTrigger <= 0.05) {
            leftRowMotor.setPower(leftTrigger);
            rightRowMotor.setPower(-leftTrigger);
        } else if (rightTrigger > 0.05 && leftTrigger <= 0.05) {
            leftRowMotor.setPower(-rightTrigger);
            rightRowMotor.setPower(rightTrigger);
        } else {
            stop();
        }
    }

    public void turretPitch(double stickY){ // stickY can be manual joystick or auto-correction from TurretAutoAim
        double delta = -stickY * PITCH_SPEED; // may be inverted
        pitchPosition += delta;
        pitchPosition = Math.max(0.0, Math.min(1.0, pitchPosition));
        setPitchPosition(pitchPosition);
    }

}

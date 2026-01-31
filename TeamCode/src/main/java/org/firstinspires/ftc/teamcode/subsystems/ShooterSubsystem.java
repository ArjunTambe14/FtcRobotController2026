package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ShooterSubsystem {

    RobotHardware r;
    SpindexerSubsystem spindexer;

    public double flywheelTargetPower = 0.8;

    enum ShotState {IDLE, PUSH, HOLD, RETURN}
    ShotState state = ShotState.IDLE;

    ElapsedTime timer = new ElapsedTime();

    public ShooterSubsystem(RobotHardware robot, SpindexerSubsystem spin){
        r = robot;
        spindexer = spin;
    }

    public void update(Gamepad g){

        boolean ready = g.left_trigger > 0.1;
        boolean shoot = g.right_trigger > 0.1;

        if(shoot && state == ShotState.IDLE){
            state = ShotState.PUSH;
            timer.reset();
        }

        switch(state){
            case IDLE:
                if(ready){
                    r.shooterFlywheel.setPower(flywheelTargetPower);
                    r.supportWheelCR.setPower(-1);
                } else {
                    r.shooterFlywheel.setPower(0);
                    r.supportWheelCR.setPower(0);
                }
                break;

            case PUSH:
                r.ballPusher.setPosition(0.8);
                state = ShotState.HOLD;
                timer.reset();
                break;

            case HOLD:
                if(timer.seconds() > 0.25){
                    state = ShotState.RETURN;
                }
                break;

            case RETURN:
                r.ballPusher.setPosition(0.5);
                spindexer.stepForward();
                state = ShotState.IDLE;
                break;
        }
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class TurretSubsystem {

    RobotHardware r;

    int centerTicks;

    public TurretSubsystem(RobotHardware robot){
        r = robot;
        centerTicks = r.turretRotate.getCurrentPosition();
    }

    public void update(Gamepad g){

        double power = g.left_stick_x * 0.5;

        r.turretRotate.setPower(power);
    }

    public double getTurretDegrees(){
        int ticks = r.turretRotate.getCurrentPosition() - centerTicks;
        return ticks / 10.0;
    }
}

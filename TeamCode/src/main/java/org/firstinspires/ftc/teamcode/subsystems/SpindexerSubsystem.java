package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class SpindexerSubsystem {

    RobotHardware r;

    int step = 0;

    double[] pos = {0.2, 0.5, 0.8};

    public SpindexerSubsystem(RobotHardware robot){
        r = robot;
    }

    public void stepForward(){
        step = (step + 1) % 3;
        r.spindexerAxon.setPosition(pos[step]);
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class IntakeSubsystem {

    RobotHardware r;

    public double intakeTargetPower = 0.7;

    public IntakeSubsystem(RobotHardware robot){
        r = robot;
    }

    public void update(Gamepad g){

        if(g.left_trigger > 0.1){
            r.intakeLeft.setPower(intakeTargetPower);
        } else r.intakeLeft.setPower(0);

        if(g.right_trigger > 0.1){
            r.intakeRight.setPower(intakeTargetPower);
        } else r.intakeRight.setPower(0);
    }
}

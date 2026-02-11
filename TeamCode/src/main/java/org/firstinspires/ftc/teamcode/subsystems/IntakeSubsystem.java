package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.EdgeButton;

public class IntakeSubsystem {

    private final RobotHardware r;
    private final double intakeStep;
    private final double triggerDeadzone;
    private final EdgeButton dpadUp = new EdgeButton();
    private final EdgeButton dpadDown = new EdgeButton();

    private double intakeTargetPower;

    public IntakeSubsystem(RobotHardware robot, double defaultPower, double intakeStep, double triggerDeadzone) {
        r = robot;
        this.intakeStep = intakeStep;
        this.triggerDeadzone = triggerDeadzone;
        this.intakeTargetPower = defaultPower;

        r.intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(Gamepad g) {
        if (dpadUp.rising(g.dpad_up)) {
            intakeTargetPower = clip01(intakeTargetPower + intakeStep);
        }
        if (dpadDown.rising(g.dpad_down)) {
            intakeTargetPower = clip01(intakeTargetPower - intakeStep);
        }

        double leftPower = (g.left_trigger > triggerDeadzone) ? intakeTargetPower : 0.0;
        double rightPower = (g.right_trigger > triggerDeadzone) ? intakeTargetPower : 0.0;

        r.intakeLeft.setPower(clip11(leftPower));
        r.intakeRight.setPower(clip11(rightPower));
    }

    public double getIntakeTargetPower() {
        return intakeTargetPower;
    }

    public void setIntakeTargetPower(double power) {
        intakeTargetPower = clip01(power);
    }

    private double clip11(double v) {
        return Range.clip(v, -1.0, 1.0);
    }

    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }
}

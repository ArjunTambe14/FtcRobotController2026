package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.EdgeButton;

public class SpindexerSubsystem {

    private final RobotHardware r;
    private final EdgeButton square = new EdgeButton();
    private final EdgeButton circle = new EdgeButton();

    private int step = 0;
    private double pos0;
    private double pos1;
    private double pos2;
    private boolean overrideActive = false;
    private double overridePos = 0.0;

    public SpindexerSubsystem(RobotHardware robot, double pos0, double pos1, double pos2) {
        r = robot;
        this.pos0 = pos0;
        this.pos1 = pos1;
        this.pos2 = pos2;
        setStep(0);
    }

    public void update(Gamepad g, boolean tuningMode) {
        boolean squarePressed = square.rising(g.x);
        boolean circlePressed = circle.rising(g.b);

        if (!tuningMode) {
            if (squarePressed) {
                stepForward();
            }
            if (circlePressed) {
                stepBackward();
            }
        }

        if (overrideActive) {
            r.spindexerAxon.setPosition(clip01(overridePos));
        } else {
            r.spindexerAxon.setPosition(clip01(getStepPosition()));
        }
    }

    public void stepForward() {
        setStep(step + 1);
    }

    public void stepBackward() {
        setStep(step - 1);
    }

    public int getStep() {
        return step;
    }

    public void setStep(int newStep) {
        step = ((newStep % 3) + 3) % 3;
        if (!overrideActive) {
            r.spindexerAxon.setPosition(clip01(getStepPosition()));
        }
    }

    public double getStepPosition() {
        switch (step) {
            case 1:
                return pos1;
            case 2:
                return pos2;
            case 0:
            default:
                return pos0;
        }
    }

    public void setPositions(double pos0, double pos1, double pos2) {
        this.pos0 = clip01(pos0);
        this.pos1 = clip01(pos1);
        this.pos2 = clip01(pos2);
        if (!overrideActive) {
            r.spindexerAxon.setPosition(clip01(getStepPosition()));
        }
    }

    public double getPos0() {
        return pos0;
    }

    public double getPos1() {
        return pos1;
    }

    public double getPos2() {
        return pos2;
    }

    public void setOverridePosition(double pos) {
        overrideActive = true;
        overridePos = clip01(pos);
        r.spindexerAxon.setPosition(clip01(overridePos));
    }

    public void clearOverride() {
        overrideActive = false;
        r.spindexerAxon.setPosition(clip01(getStepPosition()));
    }

    private double clip01(double v) {
        return Range.clip(v, 0.0, 1.0);
    }
}

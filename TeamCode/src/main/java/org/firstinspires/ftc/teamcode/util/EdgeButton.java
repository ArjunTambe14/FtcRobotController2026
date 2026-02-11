package org.firstinspires.ftc.teamcode.util;

public class EdgeButton {

    boolean last = false;

    public boolean rising(boolean now){
        boolean out = now && !last;
        last = now;
        return out;
    }
}

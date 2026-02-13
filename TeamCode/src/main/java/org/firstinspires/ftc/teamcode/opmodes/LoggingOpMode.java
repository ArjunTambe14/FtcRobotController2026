package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Storage;

public abstract class LoggingOpMode extends OpMode
{
    @Override
    public void init()
    {
        Storage.initialize();
    }

}

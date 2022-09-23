package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ExampleOpMode extends OpMode
{

    Eyes theCamera = new Eyes();

    @Override
    public void init()
    {
        theCamera.init( hardwareMap );
    }

    @Override
    public void start()
    {

    }

    @Override
    public void loop()
    {
        theCamera.stageSwitchingPipeline.getSignalZone();
    }
}

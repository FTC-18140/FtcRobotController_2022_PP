package org.firstinspires.ftc.teamcode.CommandOpModes;

import org.firstinspires.ftc.teamcode.Commands.DriveDistanceCommand;
import org.firstinspires.ftc.teamcode.Subsystems.FTClib_ThunderBot;

public class FTCLib_OpMode extends TBDOpMode
{
    FTClib_ThunderBot robot = new FTClib_ThunderBot();

    @Override
    public void init()
    {
        robot.init( hardwareMap, telemetry );
        DriveDistanceCommand step1 = new DriveDistanceCommand( 25, 0.5, robot.myChassis);
        DriveDistanceCommand step2 = new DriveDistanceCommand( 10, 0.25, robot.myChassis);
        schedule( step1, step2 );
        register(robot.myChassis);
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {

    }
}

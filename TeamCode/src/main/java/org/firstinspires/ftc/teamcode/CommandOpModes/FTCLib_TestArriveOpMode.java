package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ArriveLocationCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveDistanceCommand;
import org.firstinspires.ftc.teamcode.Subsystems.FTClib_ThunderBot;
@TeleOp(name = "Test Arrive OpMode")

public class FTCLib_TestArriveOpMode extends TBDOpModeBase
{
    FTClib_ThunderBot robot = new FTClib_ThunderBot();

    @Override
    public void init()
    {
        robot.init( hardwareMap, telemetry );

        ArriveLocationCommand step1 = new ArriveLocationCommand(10, 20, 45, 0.5, 0.1, 1, robot.myChassis, robot.myOdometry);
        schedule( step1 );
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

package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ArriveLocationCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveDistanceCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FTClib_ThunderBot;
@TeleOp(name = "Test Arrive OpMode")

public class FTCLib_TestArriveOpMode extends TBDOpModeBase
{
    ChassisSubsystem chassis = null;
    DiffOdometrySubsystem odometry = null;

    @Override
    public void init()
    {
        try
        {
            chassis = new ChassisSubsystem(hardwareMap, telemetry);
            register( chassis );

            odometry = new DiffOdometrySubsystem( chassis::getLeftEncoderDistance,
                       chassis::getRightEncoderDistance,
                       telemetry );
            register( odometry );

            ArriveLocationCommand step1 = new ArriveLocationCommand(10, 20, 45, 0.5, 0.1, 1, chassis, odometry);
            schedule( step1 );
        }
        catch (Exception e)
        {
            telemetry.addData("Something did not initialize properly.", 0);
            telemetry.addData("Ugh: ", "%s, %s, %s", e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3]);
        }
    }

    @Override
    public void init_loop()
    {
        if (odometry != null) { odometry.update(); }
        telemetry.addData("Init Loop is running... ", 1);
    }

    @Override
    public void start()
    {
    }

    @Override
    public void stop()
    {
        super.stop();
        chassis.stop();
    }
}

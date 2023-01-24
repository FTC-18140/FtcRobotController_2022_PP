package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveDistanceCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FTClib_ThunderBot;
@TeleOp(name = "Test Normal FTCLib OpMode")

public class FTCLib_OpMode extends TBDOpModeBase
{
    ChassisSubsystem chassis;
    DiffOdometrySubsystem odometry;

    @Override
    public void init()
    {
        chassis = new ChassisSubsystem(hardwareMap,
                "leftFront",
                "rightFront",
                "leftRear",
                "rightRear",
                96.0/25.4,
                telemetry);
        odometry = new DiffOdometrySubsystem( chassis::getLeftEncoderDistance,
                chassis::getRightEncoderDistance,
                15.0,
                telemetry );

        DriveDistanceCommand step1 = new DriveDistanceCommand( 25, 0.5, chassis);
        schedule( step1 );
        register(chassis, odometry);
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

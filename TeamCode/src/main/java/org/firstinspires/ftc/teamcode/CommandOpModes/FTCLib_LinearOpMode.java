package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveDistanceCommand;
import org.firstinspires.ftc.teamcode.Subsystems.FTClib_ThunderBot;

@TeleOp(name = "Test Linear OpMode")

public class FTCLib_LinearOpMode extends CommandOpMode
{
    FTClib_ThunderBot robot = new FTClib_ThunderBot();

    @Override
    public void initialize()
    {
        robot.init( hardwareMap, telemetry );
        DriveDistanceCommand step1 = new DriveDistanceCommand( 25, 0.5, robot.myChassis);
        DriveDistanceCommand step2 = new DriveDistanceCommand( 10, 0.25, robot.myChassis);
        schedule( step1, step2 );
        register(robot.myChassis);
    }
}

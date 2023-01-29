package org.firstinspires.ftc.teamcode.CommandOpModes.unused;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveDistanceCommand;
import org.firstinspires.ftc.teamcode.Subsystems.FTClib_ThunderBot;

import java.util.List;

@Autonomous(name = "Test Linear FTCLib OpMode")
@Disabled
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

    @Override
    public void reset()
    {
        robot.stop();
        super.reset();
    }
}

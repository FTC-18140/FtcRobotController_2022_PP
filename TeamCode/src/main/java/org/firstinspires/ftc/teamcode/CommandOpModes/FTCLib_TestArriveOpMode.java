package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ArriveLocationCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveDistanceCommand;
import org.firstinspires.ftc.teamcode.Commands.ElbowCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FTClib_ThunderBot;
@Autonomous(name = "FTCLib_TestArriveOpMode", group = "FTCLib")

public class FTCLib_TestArriveOpMode extends TBDOpModeBase
{
    ChassisSubsystem chassis = null;
    DiffOdometrySubsystem odometry = null;
    ArmSubsystem armstrong = null;

    @Override
    public void init()
    {
        try
        {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            chassis = new ChassisSubsystem(hardwareMap, telemetry);
            odometry = new DiffOdometrySubsystem( chassis::getLeftEncoderDistance, chassis::getRightEncoderDistance, telemetry );
            //armstrong = new ArmSubsystem(hardwareMap, telemetry);
            register( odometry, chassis, armstrong );

            ArriveLocationCommand driveAwayFromWall = new ArriveLocationCommand(100, 0, 0, 0.5, 0.1, 1, false, chassis, odometry);
            ArriveLocationCommand driveTowardsJunction = new ArriveLocationCommand( 121, 10, 45, 0.2, 0.1, 3, true, chassis, odometry);

            // Sequence the commands to drive to the junction
            //SequentialCommandGroup driveToJunction = new SequentialCommandGroup(driveAwayFromWall, driveTowardsJunction);

            // Make the command to lift up the cone away from the floor (guessing to set it at 65 degrees)
           // ElbowCommand rotateConeUp = new ElbowCommand(65, armstrong);

            // Run the elbow and chassis in parallel.
            //ParallelCommandGroup elbowAndDrive = new ParallelCommandGroup(rotateConeUp, driveToJunction);

            //schedule( elbowAndDrive ); // TODO: make sure the ArmSubsystem works with the angles and such
            //schedule( driveAwayFromWall); // for now... just drive.
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
        //if (odometry != null) { odometry.update(); }
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
        telemetry.addData("Processed stop.", 4);
    }
}

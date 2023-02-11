package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ArriveLocationCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;

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
            chassis = new ChassisSubsystem(hardwareMap, telemetry);
            odometry = new DiffOdometrySubsystem( chassis::getLeftEncoderDistance, chassis::getRightEncoderDistance, telemetry );
            //armstrong = new ArmSubsystem(hardwareMap, telemetry);
            register( chassis );
            register( odometry );
            //reigster(armstrong);

            ArriveLocationCommand driveAwayFromWall =
                    new ArriveLocationCommand(154, 90, 0.5, 0.3, 5, 0, false, false, chassis, odometry );
            ArriveLocationCommand driveTowardsJunction =
                    new ArriveLocationCommand( 167, 102, 0.5, 0.3, 2, 0, false, true, chassis, odometry );
            ArriveLocationCommand driveToCenterB =
                    new ArriveLocationCommand( 158, 90, -0.5, 0.3, 2, 0, false, false, chassis, odometry );
            ArriveLocationCommand driveToConestack =
                    new ArriveLocationCommand( 154, 31, -0.3, 0.3, 2, 0, false, true, chassis, odometry );
            ArriveLocationCommand driveToCenterF =
                    new ArriveLocationCommand( 158, 90, 0.5, 0.3, 2, 0, false, false, chassis, odometry );

            // Sequence the commands to drive to the junction and cone stack
            SequentialCommandGroup driveAroundField = new SequentialCommandGroup(
                    driveAwayFromWall,
                    driveTowardsJunction,
                    driveToCenterB,
                    driveToConestack,
                    driveToCenterF,
                    driveTowardsJunction,
                    driveToCenterB,
                    driveToConestack,
                    driveToCenterF);

            // Make the command to lift up the cone away from the floor (guessing to set it at 65 degrees)
           // ElbowCommand rotateConeUp = new ElbowCommand(65, armstrong);

            // Run the elbow and chassis in parallel.
            //ParallelCommandGroup elbowAndDrive = new ParallelCommandGroup(rotateConeUp, driveTowardsJunction);

            //schedule( elbowAndDrive ); // TODO: make sure the ArmSubsystem works with this
            schedule( driveAroundField ); // for now... just drive.
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
        //telemetry.addData("Init Loop is running... ", 1);
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

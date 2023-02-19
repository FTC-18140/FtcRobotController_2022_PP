package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ArriveLocationCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

@Autonomous(name = "FTCLib_TestArriveOpMode", group = "FTCLib")

public class FTCLib_TestArriveOpMode extends TBDOpModeBase
{
    ChassisSubsystem chassis = null;
    DiffDriveOdometrySubsystem odometry = null;
    ArmSubsystem armstrong = null;
    LiftSubsystem lift = null;

    @Override
    public void init()
    {
        try
        {
            chassis = new ChassisSubsystem(hardwareMap, telemetry);
            odometry = new DiffDriveOdometrySubsystem(chassis::getLeftEncoderDistance, chassis::getRightEncoderDistance, chassis::getHeading,
                                                      20, 90, 0, telemetry
            );
            armstrong = new ArmSubsystem(hardwareMap, telemetry);
            lift = new LiftSubsystem(hardwareMap, telemetry);

            register( chassis );
            register( odometry );
            register( armstrong );
            register( lift );

//            ArriveLocationCommand driveAwayFromWall =
//                    new ArriveLocationCommand(154, 90, 0.5, 0.3, 5, 0, false, false, chassis, odometry );
//            ArriveLocationCommand turnToJunction =
//                    new ArriveLocationCommand( 100, 100, 0.3, 0.4, 2, 40, true, false, chassis, odometry );
//            ArriveLocationCommand driveTowardsJunction =
//                    new ArriveLocationCommand( 167, 102, 0.5, 0.3, 2, 0, false, true, chassis, odometry );
//            ArriveLocationCommand driveToCenterB =
//                    new ArriveLocationCommand( 158, 90, -0.5, 0.3, 2, 0, false, false, chassis, odometry );
//            ArriveLocationCommand turnToConestack =
//                    new ArriveLocationCommand( 100, 100, 0.3, 0.4, 2, -90, true, false, chassis, odometry );
//            ArriveLocationCommand driveToConestack =
//                    new ArriveLocationCommand( 154, 31, 0.3, 0.3, 2, 0, false, true, chassis, odometry );
//            ArriveLocationCommand driveToCenterF =
//                    new ArriveLocationCommand( 158, 90, 0.5, 0.3, 2, 0, false, false, chassis, odometry );
//            ArriveLocationCommand plainTurn =
//                    new ArriveLocationCommand( 100, 100, 0.3, 0.3, 2, -90, true, false, chassis, odometry );
//
//
//            // Make the command to rotate the cone away from the floor (guessing to set it at 65 degrees)
//            ElbowCommand rotateConeUp = new ElbowCommand(65, armstrong);
//
//            // Make a command to lift the linear slide assembly up vertical.
//            LiftDistanceCommand raiseHighJunction = new LiftDistanceCommand(30, 0.5, lift);
//            LiftDistanceCommand lowerToConeStack = new LiftDistanceCommand(-20, 0.5, lift);
//            LiftDistanceCommand raiseOffConeStack = new LiftDistanceCommand(15, 0.5, lift);
//
//            // Run the elbow and chassis in parallel.
//            //ParallelCommandGroup elbowAndDriveFromWall = new ParallelCommandGroup(rotateConeUp, driveAwayFromWall);
//
//            // Drive towards the high junction and raise the lift at the same time.
//            //ParallelCommandGroup turnToAndRaiseLift = new ParallelCommandGroup( raiseHighJunction, driveTowardsJunction);
//
//            // Sequence the commands to drive to the junction and cone stack
//            SequentialCommandGroup driveAroundField = new SequentialCommandGroup(
//                    driveAwayFromWall,
//                    turnToJunction,
//                    driveTowardsJunction,
////                    elbowAndDriveFromWall,
////                    turnToAndRaiseLift,
//                    driveToCenterB ,
//                    turnToConestack,
//                    driveToConestack.withTimeout(5000));
////                    driveToCenterF,
////                    turnToAndRaiseLift,
////                    driveToCenterB,
////                   // driveToConestack.withTimeout(5000),
////                    driveToCenterF);


            ArriveLocationCommand driveAwayFromWall =
                    new ArriveLocationCommand(50, 90, 0.3, 0.3, 5, 0, false, false, chassis, odometry );
            schedule( driveAwayFromWall );
            //schedule(plainTurn);
        }
        catch (Exception e)
        {
            telemetry.addData("Something did not initialize properly.", 0);
            telemetry.addData("Ugh: ",  e.getMessage());
            telemetry.addData("Ugh2: ", "%s\n%s\n%s\n%s\n%s", e.getStackTrace()[0], e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3], e.getStackTrace()[4]);
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

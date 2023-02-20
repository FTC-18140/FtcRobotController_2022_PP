package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ArriveCommand;
import org.firstinspires.ftc.teamcode.Commands.DepartCommand;
import org.firstinspires.ftc.teamcode.Commands.SeekCommand;
import org.firstinspires.ftc.teamcode.Commands.TurnCommand;
import org.firstinspires.ftc.teamcode.Commands.WaitCommandTBD;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

@Autonomous(name = "FTCLib_TestArriveOpMode", group = "FTCLib")
@Config
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
            odometry = new DiffDriveOdometrySubsystem(chassis::getLeftEncoderDistance, chassis::getRightEncoderDistance, chassis::getHeadingAsRad,
                                                      20, 90, 0, telemetry
            );
            armstrong = new ArmSubsystem(hardwareMap, telemetry);
            lift = new LiftSubsystem(hardwareMap, telemetry);

            register( chassis );
            register( odometry );
            register( armstrong );
            register( lift );

//            ArriveCommand driveAwayFromWall =
//                    new ArriveCommand(154, 90, 0.5, 0.3, 5, 0, false, false, chassis, odometry );
//            ArriveCommand turnToJunction =
//                    new ArriveCommand( 100, 100, 0.3, 0.4, 2, 40, true, false, chassis, odometry );
//            ArriveCommand driveTowardsJunction =
//                    new ArriveCommand( 167, 102, 0.5, 0.3, 2, 0, false, true, chassis, odometry );
//            ArriveCommand driveToCenterB =
//                    new ArriveCommand( 158, 90, -0.5, 0.3, 2, 0, false, false, chassis, odometry );
//            ArriveCommand turnToConestack =
//                    new ArriveCommand( 100, 100, 0.3, 0.4, 2, -90, true, false, chassis, odometry );
//            ArriveCommand driveToConestack =
//                    new ArriveCommand( 154, 31, 0.3, 0.3, 2, 0, false, true, chassis, odometry );
//            ArriveCommand driveToCenterF =
//                    new ArriveCommand( 158, 90, 0.5, 0.3, 2, 0, false, false, chassis, odometry );
//            ArriveCommand plainTurn =
//                    new ArriveCommand( 100, 100, 0.3, 0.3, 2, -90, true, false, chassis, odometry );
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


            DepartCommand driveAwayFromWall = new DepartCommand(70, 90, 0.4, 0.1, 25, 5, false, chassis, odometry);
            ArriveCommand arriveAtJunction  = new ArriveCommand(140, 90, 0.4, 0.1, 30, 1, chassis, odometry );
            WaitCommandTBD       waitToTurn = new WaitCommandTBD(250, telemetry);
            TurnCommand      turnToJunction = new TurnCommand(45, 0.25, 0.1, .5, chassis, odometry);
            WaitCommandTBD   waitAtJunction = new WaitCommandTBD( 2000, telemetry);

            DepartCommand   driveBackwards = new DepartCommand(122, 75, -0.35, 0.3, 3, 3, false, chassis, odometry);
            WaitCommandTBD waitAfterBackup = new WaitCommandTBD(125, telemetry);

            TurnCommand     turnTowardsCones = new TurnCommand(-90, 0.3, 0.05, 2, chassis, odometry);
            WaitCommandTBD waitAfterConeTurn = new WaitCommandTBD(125, telemetry);

            SeekCommand   driveToCones1 = new SeekCommand(121, 60, 0.35, 0.35, 5, false, chassis, odometry );
            SeekCommand driveToCones1_5 = new SeekCommand(121, 40, 0.35, 0.35, 5, false, chassis, odometry );
            ArriveCommand driveToCones2 = new ArriveCommand(123, 21, 0.3, 0.3, 20, 1, chassis, odometry );
            TurnCommand    alignToCones = new TurnCommand(-90, 0.25, 0.1, 0.5, chassis, odometry);

            WaitCommandTBD waitAtCones = new WaitCommandTBD( 250, telemetry);

//            ArriveCommand driveBackwards2 =
//                    new ArriveCommand(95, 70, -0.3, 0.2, 1, chassis, odometry );
//            ArriveCommand driveToJunction =
//                    new ArriveCommand(110, 90, -0.3, 0.2, 1, chassis, odometry );
            WaitCommand waitAtJunction2 = new WaitCommand( 2000);

            SequentialCommandGroup driveAroundField = new SequentialCommandGroup(driveAwayFromWall, arriveAtJunction, waitToTurn, turnToJunction.withTimeout(1500), waitAtJunction,
                                                                                 driveBackwards, waitAfterBackup, turnTowardsCones, waitAfterConeTurn,
                                                                                 driveToCones1, driveToCones1_5, driveToCones2, waitAtCones, alignToCones);
//                                                                                 driveBackwards2, driveToJunction, waitAtJunction2);


            schedule( driveAroundField );
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

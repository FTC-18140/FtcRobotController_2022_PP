package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.ArriveCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawCommand;
import org.firstinspires.ftc.teamcode.Commands.DepartCommand;
import org.firstinspires.ftc.teamcode.Commands.ElbowCommand;
import org.firstinspires.ftc.teamcode.Commands.LiftDistanceCommand;
import org.firstinspires.ftc.teamcode.Commands.SeekCommand;
import org.firstinspires.ftc.teamcode.Commands.TurnCommand;
import org.firstinspires.ftc.teamcode.Commands.WaitCommandTBD;
import org.firstinspires.ftc.teamcode.Commands.WristCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

@Autonomous(name = "FTCLib_TestArriveOpMode", group = "FTCLib")
@Config
public class FTCLib_TestArriveOpMode extends TBDOpModeBase
{
    ChassisSubsystem chassis = null;
    DiffDriveOdometrySubsystem odometry = null;
    ArmSubsystem arm = null;
    LiftSubsystem lift = null;
    ClawSubsystem claw = null;

    @Override
    public void init()
    {
        try
        {
            chassis = new ChassisSubsystem(hardwareMap, telemetry);
            odometry = new DiffDriveOdometrySubsystem(chassis::getLeftEncoderDistance, chassis::getRightEncoderDistance, chassis::getHeadingAsRad,
                                                      20, 90, 0, telemetry );
            arm = new ArmSubsystem(hardwareMap, telemetry);
            lift = new LiftSubsystem(hardwareMap, telemetry);
            claw = new ClawSubsystem(hardwareMap, telemetry);

            register( chassis );
            register( odometry );
            register( arm );
            register( lift );
            register( claw );

            // Auto Commands

            // driveToJunction //
            ElbowCommand liftConeUp = new ElbowCommand( 0.36, arm);
            DepartCommand driveAwayFromWall = new DepartCommand(50, 90, 0.4, 0.1, 25, 5, false, chassis, odometry);
            ParallelCommandGroup driveAndElbow = new ParallelCommandGroup( liftConeUp, driveAwayFromWall);

            LiftDistanceCommand goUpToHigh = new LiftDistanceCommand(50.9, 0.75, lift);
            // ScheduleCommand raiseUpLift = new ScheduleCommand( goUpToHigh);
            ArriveCommand arriveAtJunction  = new ArriveCommand(159, 102, 0.5, 0.1, 40, 1, chassis, odometry );
            ParallelCommandGroup driveAndLift = new ParallelCommandGroup(arriveAtJunction, goUpToHigh);

            WaitCommand waitToTurn = new WaitCommand(250);
            TurnCommand turnToJunction = new TurnCommand(53.5, 0.25, 0.1, 0.75, chassis, odometry);

            SequentialCommandGroup driveToJunction = new SequentialCommandGroup( driveAndElbow, driveAndLift, waitToTurn, turnToJunction);
            //////////////////////////////////////

            // dropCone //
            WristCommand moveWristDown = new WristCommand(0, arm);
            ClawCommand openUp = new ClawCommand(0.3, claw);
            LiftDistanceCommand goDownOnHigh = new LiftDistanceCommand(-3, 0.5, lift);
            WristCommand moveWristUp = new WristCommand(0.5, arm);
            WaitCommand  waitForConeDrop = new WaitCommand( 250);

            SequentialCommandGroup dropCone = new SequentialCommandGroup( moveWristDown,  new WaitCommand(1000), goDownOnHigh, openUp, new WaitCommand(1000), moveWristUp, waitForConeDrop);
            ///////////////////////////////

            // driveToConeStack //
            LiftDistanceCommand moveLiftDown = new LiftDistanceCommand(-23, 0.5, lift);

            SeekCommand driveBack = new SeekCommand(155, 95, -0.4, 0.2, 8, true, chassis, odometry);
            TurnCommand turnTowardsCones = new TurnCommand(-90, 0.6, 0.1, 2, chassis, odometry);
            ParallelCommandGroup lowerLiftAndTurn = new ParallelCommandGroup(moveLiftDown, turnTowardsCones);

            ElbowCommand elbowDown = new ElbowCommand( 0.535, arm);
            WaitCommandTBD waitAfterConeTurn = new WaitCommandTBD(125, telemetry);

            DepartCommand driveToCones1_5 = new DepartCommand(151, 40, 0.45, 0.2, 15, 5,false, chassis, odometry );
            ArriveCommand driveToCones2 = new ArriveCommand(152, 25, 0.35, 0.2, 25, 1, chassis, odometry );
            TurnCommand alignToCones = new TurnCommand(-90, 0.25, 0.15, 0.75, chassis, odometry);
            ClawCommand grabCone = new ClawCommand(0.525, claw);

            WaitCommandTBD waitForClaw = new WaitCommandTBD( 200, telemetry);
            LiftDistanceCommand fifthConeUp = new LiftDistanceCommand(12, 0.5, lift);

            SequentialCommandGroup driveToConeStack = new SequentialCommandGroup(driveBack, lowerLiftAndTurn, elbowDown, waitAfterConeTurn, driveToCones1_5, driveToCones2, alignToCones.withTimeout(500), grabCone, waitForClaw, fifthConeUp);
            ///////////////////////////////////////

            // driveBackToJunction //
            DepartCommand  a_driveAwayCones = new DepartCommand(150, 40, -0.3, 0.2, 5, 3, false, chassis, odometry);
            SeekCommand b_midPoint = new SeekCommand(148, 75, -0.3, 0.2, 6, false, chassis, odometry);
            ArriveCommand   c_driveToCenter = new ArriveCommand( 164, 110, -0.3, 0.2, 30, 1, chassis, odometry);


            SequentialCommandGroup _1a_driveToJunction = new SequentialCommandGroup(a_driveAwayCones, b_midPoint, c_driveToCenter);

            ElbowCommand elbowBehind = new ElbowCommand( 0.28, arm);

            LiftDistanceCommand _1b_backUpHigh = new LiftDistanceCommand(30, 0.75, lift);
            WaitCommand _1c_wait = new WaitCommand(500);
            //  ScheduleCommand raiseUpAgain = new ScheduleCommand( _1b_backUpHigh );

            ParallelCommandGroup _1_liftAndDriveToCenter = new ParallelCommandGroup(_1a_driveToJunction, _1b_backUpHigh, _1c_wait.andThen(elbowBehind));

            TurnCommand   _2_alignToJunction = new TurnCommand(-128, 0.25, 0.15, 0.75, chassis, odometry);

            SequentialCommandGroup driveBacktoJunction = new SequentialCommandGroup(_1_liftAndDriveToCenter, _2_alignToJunction);
            //////////////////////////////////////////

            // dropCone2 //
            WristCommand moveWristDown2 = new WristCommand(0, arm);
            ClawCommand openUp2 = new ClawCommand(0.3, claw);
            LiftDistanceCommand goDownOnHigh2 = new LiftDistanceCommand(-3, 0.5, lift);
            WristCommand moveWristUp2 = new WristCommand(0.5, arm);
            WaitCommand  waitForConeDrop2 = new WaitCommand( 250);

            SequentialCommandGroup dropCone2 = new SequentialCommandGroup( moveWristDown2,  new WaitCommand(1000), goDownOnHigh2, openUp2, new WaitCommand(1000), moveWristUp2, waitForConeDrop2);
            ///////////////////////////////

            ////////////// MASTER COMMAND /////////
            SequentialCommandGroup driveAroundField = new SequentialCommandGroup(driveToJunction,
                                                                                 dropCone,
                                                                                 driveToConeStack,
                                                                                 driveBacktoJunction,
                                                                                 dropCone2, thirdCone());

            schedule( driveAroundField );
            ////////////////////////////
        }
        catch (Exception e)
        {
            telemetry.addData("Something did not initialize properly.", 0);
            telemetry.addData("Ugh: ",  e.getMessage());
            telemetry.addData("Ugh2: ", "%s\n%s\n%s\n%s\n%s", e.getStackTrace()[0], e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3], e.getStackTrace()[4]);
        }
    }

    private SequentialCommandGroup thirdCone() {

        LiftDistanceCommand moveLiftDown3 = new LiftDistanceCommand(-25, 0.5, lift);

        SeekCommand driveBack3 = new SeekCommand(154, 90, 0.4, 0.2, 7, true, chassis, odometry);

        ElbowCommand elbowDown3 = new ElbowCommand( 0.535, arm);
        WaitCommandTBD waitAfterConeTurn3 = new WaitCommandTBD(125, telemetry);
        DepartCommand driveToCones1_53 = new DepartCommand(145, 40, 0.4, 0.2, 15, 5,false, chassis, odometry );
        ArriveCommand driveToCones23 = new ArriveCommand(145, 25, 0.35, 0.2, 25, 1, chassis, odometry );
        TurnCommand alignToCones3 = new TurnCommand(-90, 0.25, 0.15, 0.75, chassis, odometry);
        ClawCommand grabCone3 = new ClawCommand(0.525, claw);

        WaitCommandTBD waitForClaw3 = new WaitCommandTBD( 200, telemetry);
        LiftDistanceCommand fifthConeUp3 = new LiftDistanceCommand(12, 0.5, lift);

        SequentialCommandGroup driveToConeStack3A = new SequentialCommandGroup(elbowDown3, driveBack3, waitAfterConeTurn3, driveToCones1_53, driveToCones23, alignToCones3.withTimeout(500), grabCone3, waitForClaw3);
        ParallelCommandGroup driveToConeStack3B = new ParallelCommandGroup(driveToConeStack3A, moveLiftDown3);

        SequentialCommandGroup driveAndPickUpCone = new SequentialCommandGroup(driveToConeStack3B, fifthConeUp3);
        ///////////////////////////////////////

        // driveBackToJunction //
        DepartCommand a_driveAwayCones3 = new DepartCommand(145, 40, -0.3, 0.2, 5, 3, false, chassis, odometry);
        SeekCommand b_midPoint3 = new SeekCommand(148, 75, -0.3, 0.2, 6, false, chassis, odometry);
        ArriveCommand c_driveToCenter3 = new ArriveCommand( 160, 110, -0.3, 0.2, 30, 1, chassis, odometry);


        SequentialCommandGroup _1a_driveToJunction3 = new SequentialCommandGroup(a_driveAwayCones3, b_midPoint3, c_driveToCenter3);

        ElbowCommand elbowBehind3 = new ElbowCommand( 0.28, arm);

        LiftDistanceCommand _1b_backUpHigh3 = new LiftDistanceCommand(30, 0.75, lift);
        WaitCommand _1c_wait3 = new WaitCommand(500);

        ParallelCommandGroup _1_liftAndDriveToCenter3 = new ParallelCommandGroup(_1a_driveToJunction3, _1b_backUpHigh3, _1c_wait3.andThen(elbowBehind3));

        TurnCommand   _2_alignToJunction3 = new TurnCommand(-120, 0.25, 0.15, 0.75, chassis, odometry);

        SequentialCommandGroup driveBacktoJunction3 = new SequentialCommandGroup(_1_liftAndDriveToCenter3, _2_alignToJunction3);
        //////////////////////////////////////////

        // dropCone2 //
        WristCommand moveWristDown3 = new WristCommand(0, arm);
        ClawCommand openUp3 = new ClawCommand(0.3, claw);
        LiftDistanceCommand goDownOnHigh3 = new LiftDistanceCommand(-3, 0.5, lift);
        WristCommand moveWristUp3 = new WristCommand(0.5, arm);
        WaitCommand  waitForConeDrop3 = new WaitCommand( 250);

        SequentialCommandGroup dropCone3 = new SequentialCommandGroup( moveWristDown3,  new WaitCommand(1000), goDownOnHigh3, openUp3, new WaitCommand(1000), moveWristUp3, waitForConeDrop3);
        ///////////////////////////////
        return new SequentialCommandGroup(driveAndPickUpCone, driveBacktoJunction3, dropCone3);

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
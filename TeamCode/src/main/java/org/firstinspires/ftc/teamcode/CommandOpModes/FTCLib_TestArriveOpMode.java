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
            ElbowCommand liftConeUp = new ElbowCommand( 0.375, arm);
            DepartCommand driveAwayFromWall = new DepartCommand(50, 90, 0.4, 0.1, 25, 5, false, chassis, odometry);
            ParallelCommandGroup driveAndElbow = new ParallelCommandGroup( liftConeUp, driveAwayFromWall);

            LiftDistanceCommand goUpToHigh = new LiftDistanceCommand(42, 0.5, lift);
            ScheduleCommand raiseUpLift = new ScheduleCommand( goUpToHigh);
            ArriveCommand arriveAtJunction  = new ArriveCommand(155, 97, 0.4, 0.1, 30, 1, chassis, odometry );
            ParallelCommandGroup driveAndLift = new ParallelCommandGroup(arriveAtJunction, raiseUpLift);

            WaitCommand waitToTurn = new WaitCommand(250);
            TurnCommand turnToJunction = new TurnCommand(48, 0.25, 0.1, 0.75, chassis, odometry);

            SequentialCommandGroup driveToJunction = new SequentialCommandGroup( driveAndElbow, driveAndLift, waitToTurn, turnToJunction);
            //////////////////////////////////////

            // dropCone //
            WristCommand moveWristDown = new WristCommand(0.135, arm);
            ClawCommand openUp = new ClawCommand(0.3, claw);
            LiftDistanceCommand goDownOnHigh = new LiftDistanceCommand(-3, 0.5, lift);
            WristCommand moveWristUp = new WristCommand(0.5, arm);
            WaitCommand  waitForConeDrop = new WaitCommand( 250);

            SequentialCommandGroup dropCone = new SequentialCommandGroup( moveWristDown,  new WaitCommand(1000), goDownOnHigh, openUp, new WaitCommand(1000), moveWristUp, waitForConeDrop);
            ///////////////////////////////

            // driveToConeStack //
            LiftDistanceCommand moveLiftDown = new LiftDistanceCommand(-25, 0.5, lift);
            ScheduleCommand lowerLift = new ScheduleCommand( moveLiftDown);

            TurnCommand turnTowardsCones = new TurnCommand(-90, 0.3, 0.1, 2, chassis, odometry);
            ElbowCommand elbowDown = new ElbowCommand( 0.535, arm);
            WaitCommandTBD waitAfterConeTurn = new WaitCommandTBD(125, telemetry);

            DepartCommand driveToCones1_5 = new DepartCommand(150, 40, 0.3, 0.2, 15, 5,false, chassis, odometry );
            ArriveCommand driveToCones2 = new ArriveCommand(150, 28, 0.3, 0.2, 25, 1, chassis, odometry );
            TurnCommand alignToCones = new TurnCommand(-90, 0.25, 0.15, 0.75, chassis, odometry);
            ClawCommand grabCone = new ClawCommand(0.525, claw);

            WaitCommandTBD waitForClaw = new WaitCommandTBD( 1000, telemetry);
            LiftDistanceCommand fifthConeUp = new LiftDistanceCommand(-12, 0.5, lift);

            SequentialCommandGroup driveToConeStack = new SequentialCommandGroup( lowerLift, turnTowardsCones.withTimeout(3000), elbowDown, waitAfterConeTurn, driveToCones1_5, driveToCones2, alignToCones.withTimeout(1000), grabCone, waitForClaw, fifthConeUp);
            ///////////////////////////////////////

            // driveBackToJunction //
            DepartCommand  driveAwayCones = new DepartCommand(150, 40, -0.3, 0.2, 20, 3, false, chassis, odometry);
            ArriveCommand   driveToCenter = new ArriveCommand( 155, 97, -0.3, 0.2, 20, 1, chassis, odometry);
            ElbowCommand elbowBehind = new ElbowCommand( 0.27, arm);

            LiftDistanceCommand backUpHigh = new LiftDistanceCommand(42, 0.5, lift);
            ScheduleCommand raiseUpAgain = new ScheduleCommand( backUpHigh );

            TurnCommand   alignToJunction = new TurnCommand(-132, 0.25, 0.15, 0.75, chassis, odometry);

            SequentialCommandGroup driveBackToJunction = new SequentialCommandGroup( driveAwayCones, raiseUpAgain, elbowBehind, driveToCenter, alignToJunction.withTimeout(1000));
            //////////////////////////////////////////

            // dropCone2 //
            WristCommand moveWristDown2 = new WristCommand(0.135, arm);
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
                                                                                 driveBackToJunction,
                                                                                 dropCone2);


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

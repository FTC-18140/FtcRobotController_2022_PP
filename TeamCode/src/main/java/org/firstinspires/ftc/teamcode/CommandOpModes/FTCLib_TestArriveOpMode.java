package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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

            ElbowCommand liftConeUp = new ElbowCommand( 0.375, arm);
            DepartCommand driveAwayFromWall = new DepartCommand(50, 90, 0.4, 0.1, 25, 5, false, chassis, odometry);

            ParallelCommandGroup driveAndElbow = new ParallelCommandGroup( liftConeUp, driveAwayFromWall);

            LiftDistanceCommand goUpToHigh = new LiftDistanceCommand(42, 0.5, lift);
            ArriveCommand arriveAtJunction  = new ArriveCommand(155, 97, 0.4, 0.1, 30, 1, chassis, odometry );

            ParallelCommandGroup driveAndLift = new ParallelCommandGroup(arriveAtJunction, goUpToHigh);

            WaitCommandTBD       waitToTurn = new WaitCommandTBD(250, telemetry);
            TurnCommand      turnToJunction = new TurnCommand(48, 0.25, 0.1, 0.75, chassis, odometry);

            WristCommand moveWristDown = new WristCommand(0.135, arm);
            ClawCommand openUp = new ClawCommand(0.3, claw);
            LiftDistanceCommand goDownOnHigh = new LiftDistanceCommand(-3, 0.5, lift);
            WristCommand moveWristUp = new WristCommand(0.5, arm);

            SequentialCommandGroup dropCone = new SequentialCommandGroup( moveWristDown,  new WaitCommand(1000), goDownOnHigh, openUp, new WaitCommand(1000), moveWristUp);
            WaitCommandTBD  waitForConeDrop = new WaitCommandTBD( 2000, telemetry);

            LiftDistanceCommand moveLiftDown = new LiftDistanceCommand(-43, 0.5, lift);


            DepartCommand   driveBackwards = new DepartCommand(149, 80, -0.35, 0.3, 10, 10, false, chassis, odometry);
            WaitCommandTBD waitAfterBackup = new WaitCommandTBD(125, telemetry);
            TurnCommand     turnTowardsCones = new TurnCommand(-90, 0.3, 0.1, 2, chassis, odometry);
            WaitCommandTBD waitAfterConeTurn = new WaitCommandTBD(125, telemetry);

            SeekCommand   driveToCones1 = new SeekCommand(149, 60, 0.3, 0.2, 5, false, chassis, odometry );
            SeekCommand driveToCones1_5 = new SeekCommand(149, 40, 0.3, 0.2, 5, false, chassis, odometry );
            ArriveCommand driveToCones2 = new ArriveCommand(150, 28, 0.3, 0.2, 25, 1, chassis, odometry );
            TurnCommand    alignToCones = new TurnCommand(-90, 0.25, 0.15, 0.75, chassis, odometry);
            WaitCommandTBD waitForClaw = new WaitCommandTBD( 2000, telemetry);

            DepartCommand  driveAwayCones = new DepartCommand(149, 40, -0.3, 0.2, 20, 3, false, chassis, odometry);
            ArriveCommand   driveToCenter = new ArriveCommand( 152, 90, -0.3, 0.2, 20, 1, chassis, odometry);
            TurnCommand   alignToJunction = new TurnCommand(-135, 0.25, 0.15, 0.75, chassis, odometry);

//            ArriveCommand driveToJunction =
//                    new ArriveCommand(110, 90, -0.3, 0.2, 1, chassis, odometry );
            WaitCommand waitAtJunction2 = new WaitCommand( 2000);

            SequentialCommandGroup driveAroundField = new SequentialCommandGroup(driveAndElbow, driveAndLift, waitToTurn, turnToJunction.withTimeout(1500), dropCone, waitForConeDrop, moveLiftDown.withTimeout(2000),
                                                                                 turnTowardsCones.withTimeout(3000), waitAfterConeTurn,
                                                                                 driveToCones1_5, driveToCones2, alignToCones.withTimeout(1000), waitForClaw,
                                                                                 driveAwayCones, driveToCenter, alignToJunction.withTimeout(1000));


            schedule( driveAroundField );
//            schedule( liftConeUp);

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

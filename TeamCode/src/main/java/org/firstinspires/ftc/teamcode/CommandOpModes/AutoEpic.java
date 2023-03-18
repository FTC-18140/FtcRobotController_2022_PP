package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.AprilEyes;
import org.firstinspires.ftc.teamcode.Commands.ArriveCommand;
import org.firstinspires.ftc.teamcode.Commands.ClawCommand;
import org.firstinspires.ftc.teamcode.Commands.DepartCommand;
import org.firstinspires.ftc.teamcode.Commands.ElbowCommand;
import org.firstinspires.ftc.teamcode.Commands.LiftDistanceCommand;
import org.firstinspires.ftc.teamcode.Commands.SeekCommand;
import org.firstinspires.ftc.teamcode.Commands.TurnCommand;
import org.firstinspires.ftc.teamcode.Commands.TurnToJunctionCommand;
import org.firstinspires.ftc.teamcode.Commands.WristCommand;
import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutoEpic", group = "FTCLib")
@Config
public class AutoEpic extends TBDOpModeBase
{
    ChassisSubsystem chassis = null;
    DiffDriveOdometrySubsystem odometry = null;
    ArmSubsystem arm = null;
    LiftSubsystem lift = null;
    ClawSubsystem claw = null;
    AprilEyes vision = new AprilEyes();

    Deadline timerOne = new Deadline(9, TimeUnit.SECONDS);
    Deadline timerTwo = new Deadline(19, TimeUnit.SECONDS);
    Deadline timerThree = new Deadline(27, TimeUnit.SECONDS);

    public static boolean coneOneDropped = false;
    public static boolean coneTwoDropped = false;
    public static boolean coneThreeDropped = false;

    public static DataLogger logger = new DataLogger();
    public static String logFileName = "legendaryData";

    public static double dynamicSpeed = 0.1;
    public static double dynamicTurnSpeed = 0.1;
    public static double dynamicBuffer = 1;
    public static boolean doLogging = false;

    public enum Zone {
        ONE, TWO, THREE
    }

    Zone theZone = Zone.TWO;

    @Override
    public void init()
    {
        try
        {
            if ( doLogging)
            {
                // Open file for logging
                logger.openFile(logFileName);
                logger.addField("LeftEncDist");
                logger.addField("RighEncDist");
                logger.addField("Heading");
                logger.addField("X");
                logger.addField("Y");
                logger.newLine();
            }

            vision.init(hardwareMap, telemetry);

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


            ////////////// MASTER COMMAND /////////
            SequentialCommandGroup driveAroundField = new SequentialCommandGroup(driveToJunction(),
                    dropCone(),
                    driveToConeStack());
            schedule(driveAroundField);
        }
        catch (Exception e)
        {
            telemetry.addData("Something did not initialize properly.", 0);
            telemetry.addData("Ugh: ",  e.getMessage());
            telemetry.addData("Ugh2: ", "%s\n%s\n%s\n%s\n%s", e.getStackTrace()[0], e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3], e.getStackTrace()[4]);
        }
    }

    private SequentialCommandGroup driveToJunction()
    {
        // y = 47 and a quarter inches and y = 24 and a quarter inches
        ElbowCommand liftConeUp = new ElbowCommand( 0.36, arm);

        DepartCommand halfWayFirst = new DepartCommand(70, 90, 0.4, 0.5, 20, 10, false, chassis, odometry);
        ArriveCommand otherHalfFirst = new ArriveCommand(149, 90, 0.3, 0.5, 40, 2, chassis, odometry);
        TurnCommand overshotTurn = new TurnCommand(60, 0.4, 0.1, 20, 2, chassis, odometry);
        TurnToJunctionCommand alignTurn = new TurnToJunctionCommand(false, 0.1, chassis::getFrontDistance, 45, chassis, odometry); // distThreshold = 45
        SeekCommand getCloserFirst = new SeekCommand(159, 101, 0.18, 0.4, 2, true, chassis, odometry); // x = 161 y = 100
        LiftDistanceCommand goUpToHigh = new LiftDistanceCommand(50, 0.75, lift);

        ParallelCommandGroup liftSlideAndDrive = new ParallelCommandGroup(otherHalfFirst, goUpToHigh);
        ParallelCommandGroup liftArmAndDrive = new ParallelCommandGroup(liftConeUp, halfWayFirst);

        SequentialCommandGroup firstJunction = new SequentialCommandGroup(liftArmAndDrive, liftSlideAndDrive.andThen(new WaitCommand(250)), overshotTurn, getCloserFirst, alignTurn);


        return firstJunction;
    }

    private SequentialCommandGroup dropCone()
    {
        WristCommand moveWristDown = new WristCommand(0, arm);
        ClawCommand openUp = new ClawCommand(0.3, claw, "coneOne");
        LiftDistanceCommand goDownOnHigh = new LiftDistanceCommand(-6, 0.5, lift);
        WristCommand moveWristUp = new WristCommand(0.5, arm);

        WaitCommand  waitForConeDrop = new WaitCommand( 500);
        TurnCommand correctTurnFirst = new TurnCommand(60, 0.4, 0.15, 5, 2, chassis, odometry);

        return  new SequentialCommandGroup( moveWristDown,  new WaitCommand(250), goDownOnHigh, openUp, new WaitCommand(250), moveWristUp, waitForConeDrop, correctTurnFirst);
        ///////////////////////////////
    }

    private SequentialCommandGroup driveToConeStack()
    {
        LiftDistanceCommand moveLiftDown = new LiftDistanceCommand(-20, 0.3, lift);
        ElbowCommand armDownFirst = new ElbowCommand(0.535, arm);
        SeekCommand driveBackSecond = new SeekCommand(154, 91, -0.3, 0.5, 2, true, chassis, odometry);
        TurnCommand turnToConeStackSecond = new TurnCommand(-90, 0.5, 0.2, 30, 5, chassis, odometry);

        ParallelCommandGroup slideDownAndTurnSecond = new ParallelCommandGroup(moveLiftDown, armDownFirst, turnToConeStackSecond);
        return new SequentialCommandGroup(driveBackSecond, slideDownAndTurnSecond);
    }

//    private SequentialCommandGroup driveBacktoJunction()
//    {
//
//
//
//    }
//
//    private SequentialCommandGroup dropCone2()
//    {
//
//    }
//
//    private SequentialCommandGroup thirdCone()
//    {
//
//    }
//
//    private SelectCommand goToZone()
//    {
//
//    }

    public Zone getZone()
    {
        return theZone;
    }

    @Override
    public void loop() {
        super.loop();
        if (timerOne.hasExpired()) {
            if (coneOneDropped == false) {
             //   stop();
            }
        }
        if (timerTwo.hasExpired()) {
            if (coneTwoDropped == false) {
            }
        }
        if (timerThree.hasExpired()) {
            if (coneThreeDropped == false) {

            }
        }
    }

    @Override
    public void init_loop()
    {
        int intZone = vision.getSignalZone();

        switch(intZone)
        {
            case 1:
                theZone = Zone.ONE;
                telemetry.addData("One", 0);
                break;
            case 3:
                theZone = Zone.THREE;
                telemetry.addData("Three", 0);
                break;
            default:
                theZone = Zone.TWO;
                telemetry.addData("Two", 0);
        }
        telemetry.addData("log file path", logger.getLogFullPathName());
    }


    @Override
    public void start()
    {
        timerOne.reset();
        timerTwo.reset();
        timerThree.reset();

        vision.stopCamera();
    }

    @Override
    public void stop()
    {
        super.stop();
        chassis.stop();
        lift.liftStop();
        if ( doLogging)
        {
            logger.closeDataLogger();
        }
        telemetry.addData("********************* Processed stop.*********************", 4);
    }
}
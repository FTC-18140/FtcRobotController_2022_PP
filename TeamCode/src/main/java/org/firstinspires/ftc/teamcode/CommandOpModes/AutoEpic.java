package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
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
import org.firstinspires.ftc.teamcode.Commands.TwistCommand;
import org.firstinspires.ftc.teamcode.Commands.WristCommand;
import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

import java.util.HashMap;
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
    Deadline timerTwo = new Deadline(21, TimeUnit.SECONDS);

    public static boolean coneOneDropped = false;
    public static boolean coneTwoDropped = false;

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
                    driveToConeStack(),
                    driveBacktoJunction(),
                    dropCone2(),
                    goToZone());
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
        TwistCommand initTwist = new TwistCommand(0.05, arm);
        ElbowCommand liftConeUp = new ElbowCommand( 0.64, arm);

        DepartCommand halfWayFirst = new DepartCommand(70, 90, 0.3, 0.5, 20, 10, false, chassis, odometry);
        ArriveCommand otherHalfFirst = new ArriveCommand(149, 90, 0.3, 0.5, 40, 2, chassis, odometry);
        TurnCommand overshotTurn = new TurnCommand(60, 0.4, 0.1, 20, 2, chassis, odometry);
        TurnToJunctionCommand alignTurn = new TurnToJunctionCommand(false, 0.1, chassis::getFrontDistance, 45, chassis, odometry); // distThreshold = 45
        SeekCommand getCloserFirst = new SeekCommand(159, 101, 0.18, 0.4, 2, true, chassis, odometry); // x = 161 y = 100
        LiftDistanceCommand goUpToHigh = new LiftDistanceCommand(50, 0.75, lift);

        ParallelCommandGroup liftSlideAndDrive = new ParallelCommandGroup(otherHalfFirst, goUpToHigh);
        ParallelCommandGroup liftArmAndDrive = new ParallelCommandGroup(liftConeUp, halfWayFirst);

        SequentialCommandGroup firstJunction = new SequentialCommandGroup(initTwist, liftArmAndDrive, liftSlideAndDrive.andThen(new WaitCommand(250)), overshotTurn, getCloserFirst.withTimeout(2000), alignTurn);

        return firstJunction;
    }

    private SequentialCommandGroup dropCone()
    {
        WristCommand moveWristDown = new WristCommand(0, arm);
        ClawCommand openUp = new ClawCommand(0.3, claw, "coneOne");

        LiftDistanceCommand goDownOnHigh = new LiftDistanceCommand(-6, 0.5, lift);
        WristCommand moveWristUp = new WristCommand(0.5, arm);
        TwistCommand twistCorrection = new TwistCommand(0.05, arm);

        WaitCommand  waitForConeDrop = new WaitCommand( 500);
        TurnCommand correctTurnFirst = new TurnCommand(60, 0.4, 0.15, 5, 2, chassis, odometry);

        return  new SequentialCommandGroup( new TwistCommand(0.05, arm), moveWristDown,  new WaitCommand(500), twistCorrection.andThen(new WaitCommand(500)), goDownOnHigh, openUp, new WaitCommand(250), moveWristUp, waitForConeDrop, correctTurnFirst);
        ///////////////////////////////
    }

    private SequentialCommandGroup driveToConeStack()
    {
        LiftDistanceCommand moveLiftDown = new LiftDistanceCommand(-21, 0.3, lift);
        ElbowCommand armDownFirst = new ElbowCommand(0.465, arm);
        SeekCommand driveBackSecond = new SeekCommand(152, 90, -0.35, 0.5, 2, true, chassis, odometry);
        TurnCommand turnToConeStackSecond = new TurnCommand(-90, 0.5, 0.2, 30, 5, chassis, odometry);

        DepartCommand driveHalfToConeStack = new DepartCommand(152, 40, 0.4, 0.40, 8, 2, false, chassis, odometry);
        ArriveCommand driveOtherHalf = new ArriveCommand(152, 25, 0.2, 0.4, 10, 2, chassis, odometry);

        ClawCommand clawCloseSecond = new ClawCommand(0.525, claw, "none");
        LiftDistanceCommand liftSlightlySecond = new LiftDistanceCommand(12, 0.5, lift);

        ParallelCommandGroup slideDownAndTurnSecond = new ParallelCommandGroup(moveLiftDown, armDownFirst, turnToConeStackSecond);
        return new SequentialCommandGroup(driveBackSecond, slideDownAndTurnSecond, driveHalfToConeStack, driveOtherHalf.withTimeout(2000), clawCloseSecond.andThen(new WaitCommand(250)), liftSlightlySecond.andThen(new WaitCommand(250)));

    }

    private SequentialCommandGroup driveBacktoJunction()
    {
        SeekCommand driveBackToJunctionSecond = new SeekCommand(152, 45, -0.3, 0.4, 8, false, chassis, odometry);
        ArriveCommand driveRestOfWayToJunctionSecond = new ArriveCommand(152, 90, -0.3, 0.4, 30, 2, chassis, odometry);

        LiftDistanceCommand liftSlideSecond = new LiftDistanceCommand(18, 0.5, lift);
        ElbowCommand elbowRaiseHalfSecond = new ElbowCommand(0.6, arm);
        ElbowCommand elbowRaiseSecond = new ElbowCommand(0.73, arm);

        TurnCommand overShotSecond = new TurnCommand(-120, 0.4, 0.2, 15, 2, chassis, odometry);

        SeekCommand getCloserSecond = new SeekCommand(152, 88.5, -0.2, 0.4, 8, true, chassis, odometry);
        TurnToJunctionCommand alignForSecondCone = new TurnToJunctionCommand(false, 0.15, chassis::getBackDistance, 10, chassis, odometry);
        SeekCommand getCloserSecond2 = new SeekCommand(160, 108, -0.18, 0.4, 2, true, chassis, odometry); // x = 161 y = 100

        ParallelCommandGroup liftAndDriveSecond = new ParallelCommandGroup(driveRestOfWayToJunctionSecond, liftSlideSecond, elbowRaiseHalfSecond.andThen(new WaitCommand(500).andThen(elbowRaiseSecond)));

        return new SequentialCommandGroup(driveBackToJunctionSecond, liftAndDriveSecond, overShotSecond, getCloserSecond, getCloserSecond2.withTimeout(2000), alignForSecondCone);
    }

    private SequentialCommandGroup dropCone2()
    {
        WristCommand wristMoveSecond = new WristCommand(0, arm);
        ClawCommand clawMoveDownSecond = new ClawCommand(0.3, claw, "none");
        ClawCommand clawCloseSecond = new ClawCommand(0.36, claw, "none");
        LiftDistanceCommand goDownOnHighSecond = new LiftDistanceCommand(-6, 0.5, lift);
        WristCommand moveWristUpSecond = new WristCommand(0.5, arm);
        TwistCommand twistCorrectionSecond = new TwistCommand(1, arm);

        TurnCommand correctTurnThird = new TurnCommand(-120, 0.4, 0.2, 15, 2, chassis, odometry);

        return new SequentialCommandGroup(twistCorrectionSecond, wristMoveSecond.andThen(new WaitCommand(500)), goDownOnHighSecond.andThen(new WaitCommand(250)), clawMoveDownSecond.andThen(new WaitCommand(250).andThen(clawCloseSecond)), moveWristUpSecond, correctTurnThird);
    }

    private SelectCommand goToZone()
    {
        SeekCommand goToZone1a = new SeekCommand( 152, 95, 0.2, 0.3, 5,true, chassis, odometry );
        TurnCommand goToZone1b = new TurnCommand(-90, 0.3, 0.2, 10, 3, chassis, odometry);
        SeekCommand goToZone1c = new SeekCommand(152, 35, 0.2, 0.3, 5, true, chassis, odometry);
        LiftDistanceCommand liftDownOne = new LiftDistanceCommand(-40, 0.5, lift);
        ElbowCommand elbowDownOne = new ElbowCommand(0.495, arm);

        //
        SeekCommand goToZone2a = new SeekCommand( 152, 95, 0.2, 0.3, 5,true, chassis, odometry );
        TurnCommand goToZone2b = new TurnCommand(-90, 0.3, 0.2, 10, 3, chassis, odometry);
        LiftDistanceCommand liftDownTwo = new LiftDistanceCommand(-45, 0.5, lift);
        ElbowCommand elbowDownTwo = new ElbowCommand(0.495, arm);


        SeekCommand goToZone3a = new SeekCommand( 152, 95, 0.2, 0.3, 5,true, chassis, odometry );
        TurnCommand goToZone3b = new TurnCommand(-90, 0.3, 0.2, 10, 3, chassis, odometry);
        SeekCommand goToZone3c = new SeekCommand(152, 155, -0.2, 0.3, 5, true, chassis, odometry);
        LiftDistanceCommand liftDownThree = new LiftDistanceCommand(-40, 0.5, lift);
        ElbowCommand elbowDownThree = new ElbowCommand(0.495, arm);

        ParallelCommandGroup zoneOne = new ParallelCommandGroup(goToZone1a.andThen(goToZone1b.andThen(goToZone1c)), liftDownOne, elbowDownOne);
        ParallelCommandGroup zoneTwo = new ParallelCommandGroup(goToZone2a.andThen(goToZone2b), liftDownTwo, elbowDownTwo);
        ParallelCommandGroup zoneThree = new ParallelCommandGroup(goToZone3a.andThen(goToZone3b.andThen(goToZone3c)), liftDownThree, elbowDownThree);


        // This HashMap lets us associate the signal zone with an command group.
        HashMap<Object, Command> theMap =  new HashMap<Object, Command>();
        theMap.put( AutoEpic.Zone.ONE, zoneOne);
        theMap.put(AutoEpic.Zone.TWO, zoneTwo);
        theMap.put(AutoEpic.Zone.THREE, zoneThree);

        return new SelectCommand(theMap,this::getZone);
    }

    public Zone getZone()
    {
        return theZone;
    }

    @Override
    public void loop() {
        super.loop();
        if (timerOne.hasExpired()) {
            if (coneOneDropped == false) {
         //       stop();
            }
        }
        if (timerTwo.hasExpired()) {
            if (coneTwoDropped == false) {
           //     stop();
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
                theZone = Zone.ONE; // ONE
                telemetry.addData("One", 0);
                break;
            case 3:
                theZone = Zone.THREE; // THREE
                telemetry.addData("Three", 0);
                break;
            default:
                theZone = Zone.TWO; // TWO
                telemetry.addData("Two", 0);
        }
        telemetry.addData("log file path", logger.getLogFullPathName());
    }


    @Override
    public void start()
    {
        timerOne.reset();
        timerTwo.reset();

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
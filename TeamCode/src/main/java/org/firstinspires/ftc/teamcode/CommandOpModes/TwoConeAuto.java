package org.firstinspires.ftc.teamcode.CommandOpModes;

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
import org.firstinspires.ftc.teamcode.Commands.WristCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

@Autonomous(name = "TwoConeAuto", group = "FTCLib")
public class TwoConeAuto extends TBDOpModeBase
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

            SequentialCommandGroup driveAroundField = new SequentialCommandGroup(driveToJunction(0.4),
                                                                                 dropCone(),
                                                                                 driveToConeStack(0.4),
                                                                                 driveBackToJunction(0.4),
                                                                                 dropCone());


            schedule( driveAroundField );
        }
        catch (Exception e)
        {
            telemetry.addData("Something did not initialize properly.", 0);
            telemetry.addData("Ugh: ",  e.getMessage());
            telemetry.addData("Ugh2: ", "%s\n%s\n%s\n%s\n%s", e.getStackTrace()[0], e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3], e.getStackTrace()[4]);
        }
    }

    private SequentialCommandGroup driveToJunction( double speed)
    {
        // Define the commands to get the arm and linear slides set up
        ElbowCommand _1a_ConeUp = new ElbowCommand( 0.375, arm);
        LiftDistanceCommand _1b_raiseToHigh = new LiftDistanceCommand(49, 0.5, lift);

        // Define the path to drive to the junction
        DepartCommand _a_depart = new DepartCommand(52, 87, speed, 0.1, 25, 5, false, chassis, odometry);
        SeekCommand _b_midPoint = new SeekCommand(82, 82, speed, 0.3, 3, false, chassis, odometry);
        SeekCommand _c_midPoint = new SeekCommand(105, 79, speed, 0.3, 3, false, chassis, odometry);
        SeekCommand _d_midPoint = new SeekCommand(137, 85, speed, 0.3, 3, false, chassis, odometry);
        ArriveCommand _e_arriveAtJunction  = new ArriveCommand(151, 94, speed, 0.1, 40, 1, chassis, odometry );

        SequentialCommandGroup _1c_drivePath = new SequentialCommandGroup( _a_depart, _b_midPoint, _c_midPoint, _d_midPoint, _e_arriveAtJunction);

        ParallelCommandGroup driveToPole = new ParallelCommandGroup( _1a_ConeUp, _1b_raiseToHigh, _1c_drivePath );
        TurnCommand alignToPole = new TurnCommand(45, 0.25, 0.15, 0.75, chassis, odometry);

        return new SequentialCommandGroup( driveToPole, alignToPole.withTimeout(250));
    }

    private SequentialCommandGroup dropCone()
    {
        // Move wrist into position over junction
        WristCommand _1_moveWristDown = new WristCommand(0.135, arm);
        WaitCommand _2_wait = new WaitCommand(250);

        // Slide down on junction and open claw to drop cone
        LiftDistanceCommand slideDown3Inches = new LiftDistanceCommand(-3, 0.5, lift);
        WaitCommand  slightPause = new WaitCommand( 250);
        ClawCommand openClaw = new ClawCommand(0.3, claw);
        ParallelCommandGroup _3_coneDownClawOpen = new ParallelCommandGroup(slideDown3Inches, slightPause.andThen(openClaw));

        // Move wrist back out of the way
        WristCommand _4_moveWristUp = new WristCommand(0.5, arm);

        return new SequentialCommandGroup( _1_moveWristDown, _2_wait, _3_coneDownClawOpen, _4_moveWristUp);
    }

    private SequentialCommandGroup driveToConeStack( double speed )
    {
        LiftDistanceCommand downToStackHeight = new LiftDistanceCommand(-17, 0.5, lift);
        TurnCommand turnTowardsCones = new TurnCommand(-90, 0.3, 0.1, 2, chassis, odometry);
        ElbowCommand elbowDown = new ElbowCommand( 0.535, arm);
        WaitCommand slightPause = new WaitCommand(500);

        ParallelCommandGroup _1_lowerLiftAndTurn = new ParallelCommandGroup(downToStackHeight, turnTowardsCones, slightPause.andThen(elbowDown));

        DepartCommand _a_depart = new DepartCommand(150, 72, speed, 0.4, 10, 5,false, chassis, odometry );
        SeekCommand _b_midPoint = new SeekCommand(150, 58, speed,0.4, 2, false, chassis, odometry);
        SeekCommand _c_midPoint = new SeekCommand(150, 38, speed, 0.3, 2, false, chassis, odometry);
        ArriveCommand _d_arrive = new ArriveCommand(150, 25, speed, 0.2, 10, 1, chassis, odometry );
        TurnCommand _e_align = new TurnCommand(-90, 0.25, 0.15, 0.75, chassis, odometry);

        SequentialCommandGroup _2_driveToStack = new SequentialCommandGroup(_a_depart, _b_midPoint, _c_midPoint, _d_arrive, _e_align.withTimeout(500));

        ClawCommand _3_grabCone = new ClawCommand(0.525, claw);
        WaitCommand pause = new WaitCommand( 250 );

        LiftDistanceCommand _4_fifthConeUp = new LiftDistanceCommand(12, 0.5, lift);

        return new SequentialCommandGroup( _1_lowerLiftAndTurn,  _2_driveToStack, _3_grabCone.andThen(pause), _4_fifthConeUp);
    }

    public SequentialCommandGroup driveBackToJunction( double speed )
    {
        DepartCommand  _a_depart = new DepartCommand(150, 38, -1*speed, 0.2, 5, 3, false, chassis, odometry);
        SeekCommand _b_midPoint = new SeekCommand(150, 58, -1*speed,0.2, 2, false, chassis, odometry);
        SeekCommand _c_midPoint = new SeekCommand(157, 74, -1*speed,0.2, 2, false, chassis, odometry);
        ArriveCommand   _d_arrive = new ArriveCommand( 161, 103, -1*speed, 0.2, 30, 1, chassis, odometry);

        SequentialCommandGroup _1a_driveBack = new SequentialCommandGroup(_a_depart, _b_midPoint, _c_midPoint, _d_arrive);
        ElbowCommand _1b_elbowBack = new ElbowCommand( 0.27, arm);
        LiftDistanceCommand _1c_backUpHigh = new LiftDistanceCommand(49, 0.5, lift);
        WaitCommand pause = new WaitCommand( 250 );

        ParallelCommandGroup _1_liftAndDriveToPole = new ParallelCommandGroup(_1a_driveBack, pause.andThen(_1b_elbowBack), _1c_backUpHigh);

        TurnCommand _2_alignToPole = new TurnCommand(-128, 0.25, 0.15, 0.75, chassis, odometry);

        return new SequentialCommandGroup(_1_liftAndDriveToPole, _2_alignToPole.withTimeout(500));
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

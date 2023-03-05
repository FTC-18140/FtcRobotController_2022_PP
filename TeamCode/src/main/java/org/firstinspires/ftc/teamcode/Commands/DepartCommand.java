package org.firstinspires.ftc.teamcode.Commands;


import com.arcrobotics.ftclib.hardware.motors.Motor;


import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

import java.util.concurrent.TimeUnit;

public class DepartCommand extends DriveCommand
{
    private final double myLaunchZoneCM;
    private Deadline startTimer = new Deadline(5000,TimeUnit.MILLISECONDS);
    private double timeToAccel = 1.25;

    /**
     * Creates a new ArriveCommand.
     *
     */
    public DepartCommand(double x, double y, double speed, double turnSpeed, double launchZoneCM, double targetZoneCM, boolean stopAtEnd, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {
        super(x, y, speed, turnSpeed, targetZoneCM, stopAtEnd, chassis, odometry);
        myLaunchZoneCM = launchZoneCM;
        myMotionProfile.setAccelBufferCM(launchZoneCM);

    }

    @Override
    public void initialize()
    {
        super.initialize();
        telemetry.addData("Depart Command Initialized,", toPoint);
        myChassisSubsystem.setZeroBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void execute()
    {
        if(firstExecute)
        {
            startTimer.reset();
        }

        if ( myBackwards)
        {
            telemetry.addData("Executing Depart Command Backwards: ", "%.2f, %.2f", toPoint.getX(), toPoint.getY());
        }
        else
        {
            telemetry.addData("Executing Depart Command: ", "%.2f, %.2f", toPoint.getX(), toPoint.getY());
        }
        super.execute();
    }

    /**
     * Adjusts the motor speeds based on this path's motion profile.
     *
     * @param speeds       Speeds to be adjusted.
     */
    @Override
    public void profileMotorPowers(double[] speeds)
    {
        double accel = mySpeed/timeToAccel;

        if ( !startTimer.hasExpired())
        {
            speeds[0] = accel * startTimer.timeRemaining(TimeUnit.MILLISECONDS)/1000;
        }
        
//        if (fromDistance < myLaunchZoneCM )
//        {    // If the robot is closer to the "from" point, do acceleration
//            myMotionProfile.processAccelerate(speeds, fromDistance, mySpeed, myTurnSpeed);
//
//        }
        myMotionProfile.processHeading(speeds, relativeAngleToPosition, myTurnSpeed);
    }

}

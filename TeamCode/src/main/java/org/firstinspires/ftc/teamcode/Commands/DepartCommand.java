package org.firstinspires.ftc.teamcode.Commands;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

import java.util.concurrent.TimeUnit;
@Config
public class DepartCommand extends DriveCommandBase
{
    private final double myLaunchZoneCM;
    private final ElapsedTime accelerationTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static public double timeToAccel = 0.8; // seconds

    /**
     * Creates a new ArriveCommand.
     *
     */
    public DepartCommand(double x, double y, double speed, double turnSpeed, double departZoneCM, double arriveBufferCM, boolean stopAtEnd, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {
        super(x, y, speed, turnSpeed, arriveBufferCM, stopAtEnd, chassis, odometry);
        myLaunchZoneCM = departZoneCM;
        myMotionProfile.setAccelBufferCM(departZoneCM);

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
            accelerationTimer.reset();
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
        double time = accelerationTimer.time(TimeUnit.MILLISECONDS) / 1000.0; //seconds

        if ( time <= timeToAccel)
        {
            speeds[0] = accel * time;
        }
        else
        {
            speeds[0] = mySpeed;
        }

        myMotionProfile.processHeading(speeds, relativeAngleToPosition, myTurnSpeed);
    }

}

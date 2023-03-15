package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.MovingAverage;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

@Config
public class ArriveCommand extends DriveCommandBase
{
    private final double mySlowDownZoneCM;
    public static int filterSize = 30;
    private MovingAverage decelFilter = new MovingAverage(filterSize);

    /**
     * Creates a new ArriveCommand.
     *
     */
    public ArriveCommand(double x, double y, double speed, double turnSpeed, double arriveZoneCM, double arriveBufferCM, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {

        // Need to use the arriveBuffer in the super class where endZoneCM is normally used.
        super(x, y, speed, turnSpeed, arriveBufferCM, true, chassis, odometry);
        mySlowDownZoneCM = arriveZoneCM;
        myMotionProfile.setDecelBufferCM(arriveZoneCM);

    }

    @Override
    public void initialize()
    {
        super.initialize();
        telemetry.addData("Arrive Command Initialized,", toPoint);
        myChassisSubsystem.setZeroBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void execute()
    {
        if ( myBackwards)
        {
            telemetry.addData("Executing Arrive Command Backwards: ", "%.2f, %.2f", toPoint.getX(), toPoint.getY());
        }
        else
        {
            telemetry.addData("Executing Arrive Command: ", "%.2f, %.2f", toPoint.getX(), toPoint.getY());
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
       if (toDistance < mySlowDownZoneCM)
        {    // If the robot is closer to the "to" point, do deceleration
            myMotionProfile.processDecelerate(speeds, toDistance, mySpeed, myTurnSpeed);
            decelFilter.add(speeds[0]);
            speeds[0] = decelFilter.getValue();
        }
        myMotionProfile.processHeading(speeds, relativeAngleToPosition, myTurnSpeed);
    }


}

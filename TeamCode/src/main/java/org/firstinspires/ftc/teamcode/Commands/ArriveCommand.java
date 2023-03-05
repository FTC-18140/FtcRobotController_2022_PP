package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

import static java.lang.Math.abs;
import static java.lang.Math.ulp;

public class ArriveCommand extends DriveCommandBase
{
    private final double mySlowDownZoneCM;

    /**
     * Creates a new ArriveCommand.
     *
     */
    public ArriveCommand(double x, double y, double speed, double turnSpeed, double endZoneCM, double arriveBuffer, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {

        // Need to use the arriveBuffer in the super class where endZoneCM is normally used.
        super(x, y, speed, turnSpeed, arriveBuffer, true, chassis, odometry);
        mySlowDownZoneCM = endZoneCM;
        myMotionProfile.setDecelBufferCM(arriveBuffer);

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
        }
        myMotionProfile.processHeading(speeds, relativeAngleToPosition, myTurnSpeed);
    }


}

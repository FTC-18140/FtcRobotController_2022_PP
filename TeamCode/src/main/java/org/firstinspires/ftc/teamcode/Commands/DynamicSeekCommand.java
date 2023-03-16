package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

public class DynamicSeekCommand extends DriveCommandBase
{

    /**
     * Creates a new SeekCommand.
     *
     */
    public DynamicSeekCommand(double x, double y, double speed, double turnSpeed, double arriveBufferCM, boolean stopAtEnd, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {
        super(x, y, speed, turnSpeed, arriveBufferCM, stopAtEnd, chassis, odometry);
    }

    @Override
    public void initialize()
    {
        super.initialize();
        // 20.32 is the distance from the pole that is required to place a cone
        double dX = (myChassisSubsystem.getFrontDistance() - 20.32) * Math.cos(myOdometrySubsystem.getPose().getHeading());
        double dY = (myChassisSubsystem.getFrontDistance() - 20.32) * Math.sin(myOdometrySubsystem.getPose().getHeading());
        double newX = myOdometrySubsystem.getPose().getX() + dX;
        double newY = myOdometrySubsystem.getPose().getY() + dY;
        toPoint = new Translation2d(newX, newY);
        telemetry.addData("Seek Command Initialized,", toPoint);
        myChassisSubsystem.setZeroBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void execute()
    {
        if ( myBackwards)
        {
            telemetry.addData("Executing Seek Command Backwards: ", "%.2f, %.2f", toPoint.getX(), toPoint.getY());
        }
        else
        {
            telemetry.addData("Executing Seek Command: ", "%.2f, %.2f", toPoint.getX(), toPoint.getY());
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
        myMotionProfile.processHeading(speeds, relativeAngleToPosition, myTurnSpeed);
    }


}

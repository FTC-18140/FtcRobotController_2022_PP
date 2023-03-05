package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

public class SeekCommand extends DriveCommandBase
{
//    private final ChassisSubsystem myChassisSubsystem;
//    private final DiffDriveOdometrySubsystem myOdometrySubsystem;
//    private final MotionProfile myMotionProfile;
//
//    private Translation2d fromPoint;
//    private final Translation2d toPoint;
////    private final double toHeading;
//
//    private Pose2d myRobotPose;
//
//    private final double myEndZoneCM;
//    private final double mySpeed;
//    private final double myTurnSpeed;
//
//    private boolean myArrived = false;
//
//    private boolean myStopAtEnd;
//    private final boolean myBackwards;
//
//    // Delta values.
//    double toDeltaX;
//    double toDeltaY;
//
//    // Distances
//    double fromDistance;
//    double toDistance;
//    double absoluteAngleToPosition;
//    double relativeAngleToPosition;
//
//    Telemetry telemetry;
//    private boolean firstExecute = true;

    /**
     * Creates a new ArriveCommand.
     *
     */
    public SeekCommand(double x, double y, double speed, double turnSpeed, double endZoneCM, boolean stopAtEnd, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {
        super( x, y, speed, turnSpeed, endZoneCM, stopAtEnd, chassis, odometry);
    }

    @Override
    public void initialize()
    {
        super.initialize();
        telemetry.addData("Seek Command Initialized,", toPoint);
        myChassisSubsystem.setZeroBehavior(Motor.ZeroPowerBehavior.FLOAT);
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

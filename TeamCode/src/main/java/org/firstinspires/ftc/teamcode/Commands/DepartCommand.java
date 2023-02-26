package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

public class DepartCommand extends CommandBase
{
    private final ChassisSubsystem myChassisSubsystem;
    private final DiffDriveOdometrySubsystem myOdometrySubsystem;
    private final MotionProfile myMotionProfile;

    private Translation2d fromPoint;
    private final Translation2d toPoint;
//    private final double toHeading;

    private Pose2d myRobotPose;

    private final double myTargetZoneCM;
    private final double myLaunchZoneCM;
    private final double mySpeed;
    private final double myTurnSpeed;

    private boolean myArrived = false;

    private final boolean myStopAtEnd;
//    private final boolean myTurnOnly;
    private final boolean myBackwards;

    // Delta values.
    double fromDeltaX;
    double fromDeltaY;
    double toDeltaX;
    double toDeltaY;

    // Distances
    double fromDistance;
    double toDistance;
    double absoluteAngleToPosition;
    double relativeAngleToPosition;

    private boolean firstExecute = true;

    Telemetry telemetry;

    /**
     * Creates a new ArriveCommand.
     *
     */
    public DepartCommand(double x, double y, double speed, double turnSpeed, double launchZoneCM, double targetZoneCM, boolean stopAtEnd, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {
        toPoint = new Translation2d(x, y);
//        toHeading = Math.toRadians(heading);
        mySpeed = speed;
        myBackwards = (speed < 0);
        myTurnSpeed = turnSpeed;
        myTargetZoneCM = targetZoneCM;
        myLaunchZoneCM = launchZoneCM;
        myChassisSubsystem = chassis;
        myOdometrySubsystem = odometry;
        telemetry = myChassisSubsystem.getTelemetry();
//        myTurnOnly = turnOnly;
        myStopAtEnd = stopAtEnd;

        myMotionProfile = new MotionProfile(myLaunchZoneCM, myTargetZoneCM, 1, 0.005);
        // temp
        myMotionProfile.telem = telemetry;

        addRequirements(myChassisSubsystem, myOdometrySubsystem);
    }

    @Override
    public void initialize()
    {
        telemetry.addData("Depart Command Initialized,", toPoint);
    }

    @Override
    public void execute()
    {
        if (firstExecute)
        {
            fromPoint = new Translation2d(myOdometrySubsystem.getPose().getTranslation().getX(), myOdometrySubsystem.getPose().getTranslation().getY());
            firstExecute = false;
        }
        if ( myBackwards)
        {
            telemetry.addData("Executing Depart Command Backwards: ", "%.2f, %.2f", toPoint.getX(),
                              toPoint.getY());
        }
        else
        {
            telemetry.addData("Executing Depart Command: ", "%.2f, %.2f", toPoint.getX(),
                              toPoint.getY());
        }
        // Get new position data from the odometry subsystem and update the robot's location and
        // distance/angles relative to the From Point and the To Point.
        double driveDistance = updatePositions();
        telemetry.addData("Distance to Point: ", driveDistance);

        // Now that the robot's position is updated. Figure out the angle the robot needs to
        // drive in order to get to the To Point.
        double turnAngle = updateAngles();

        // Determine if robot needs to drive and turn to get to the position.
        double[] motorPowers = new double[3];
        motorPowers[1] = 0; // no strafing

//        if (myTurnOnly)
//        { // no translation speed
//            telemetry.addData("RobotHeading: ", Math.toDegrees(myRobotPose.getHeading()));
//            telemetry.addData("myHeading: ", Math.toDegrees(toHeading));
//            motorPowers[0] = 0.0;
//            motorPowers[2] = Range.clip(-turnAngle, -1.0 * myTurnSpeed, myTurnSpeed);
//            MotionProfile.headingMinSpeed = 0.08;
//
//        }
//        else
        {  // find translation speed
            if (myBackwards)
            {
                motorPowers[0] = Range.clip(-driveDistance, mySpeed, -0.1);
                telemetry.addData("Motor p 0: ", "%.2f --> %.2f", driveDistance, motorPowers[0]);

            }
            else
            {
                motorPowers[0] = Range.clip(driveDistance, 0.1, mySpeed);
            }
            motorPowers[2] = Range.clip(-turnAngle, -1.0 * myTurnSpeed, myTurnSpeed);

            myMotionProfile.setMinTurnSpeed(0.03 * Math.abs(turnAngle)/Math.toRadians(10));

        }

        // Do the motion profiling on the motor powers based on where we are relative to the target
        profileMotorPowers(motorPowers);

        telemetry.addData("Power 0, profiled: ", motorPowers[0]);
        telemetry.addData("Power 2, profiled: ", motorPowers[2]);

        // Check if we have arrived
//        if ( myTurnOnly )
//        { // arriving on a Turn Only move is based on closeness to myHeading
//            myArrived = rotationEqualsWithBuffer(myRobotPose.getHeading(), toHeading, Math.toRadians(myBuffer));
//        }
//        else
        { // arriving on a normal move is based on closeness to the toPoint
            myArrived = positionEqualsWithBuffer(myRobotPose.getTranslation(), toPoint,
                                                 myTargetZoneCM);
        }

        telemetry.addData("Arrived at toPoint?  ", myArrived);
        if (myArrived)
        {
            motorPowers[0] = 0;
            motorPowers[1] = 0;
            motorPowers[2] = 0;
        }

        if ((Math.abs(motorPowers[0])+Math.abs(motorPowers[2])) > myTurnSpeed )
        {
            motorPowers[0] = motorPowers[0] * Math.abs(motorPowers[0]/(Math.abs(motorPowers[0])+Math.abs(motorPowers[2])));
        }
        myChassisSubsystem.arcadeDrive(motorPowers[0], motorPowers[2]);
    }

    private double updatePositions()
    {
        // Update the current position of the Robot by getting it from the odometry subsystem.
        myRobotPose = myOdometrySubsystem.getPose();

        // Update delta Translation values so that we can update how far the robot is from the
        // From Point and the To Point
        fromDeltaX = myRobotPose.getTranslation().getX() - fromPoint.getX();
        fromDeltaY = myRobotPose.getTranslation().getY() - fromPoint.getY();

        toDeltaX = toPoint.getX() - myRobotPose.getTranslation().getX();
        toDeltaY = toPoint.getY() - myRobotPose.getTranslation().getY();
        telemetry.addData("dX2Point, dY2Point: ", "%.3f, %.3f", toDeltaX,toDeltaY);

        // Update distances between Robot and From Point and Robot and To Point
        fromDistance = Math.hypot(fromDeltaX, fromDeltaY);
        toDistance = Math.hypot(toDeltaX, toDeltaY);
        return toDistance;
    }

    private double updateAngles()
    {
        // Update the angle to To point in absolute, field-based perspective.  This represents the
        // the angle the robot needs to move to get to the To Point.
        absoluteAngleToPosition = Math.atan2(toDeltaY, toDeltaX);
        telemetry.addData("AbsAngle, deg: ", Math.toDegrees(absoluteAngleToPosition));

        // Based on the current heading of the robot, update the value that determines how much
        // the robot needs to turn
//        if (myTurnOnly)
//        {
//            relativeAngleToPosition = -angleWrap(toHeading - myRobotPose.getHeading());
//        }
//        else
        {
            relativeAngleToPosition = -angleWrap(absoluteAngleToPosition - myRobotPose.getHeading());

            if (myBackwards)
            {
                relativeAngleToPosition = angleWrap(relativeAngleToPosition - Math.PI);
            }
        }
        telemetry.addData("Relative Angle, deg: ", Math.toDegrees(relativeAngleToPosition));
        return relativeAngleToPosition;
    }

    /**
     * Adjusts the motor speeds based on this path's motion profile.
     *
     * @param speeds       Speeds to be adjusted.
     */
    private void profileMotorPowers(double[] speeds)
    {
        if (fromDistance < myLaunchZoneCM )
        {    // If the robot is closer to the "from" point, do acceleration
            myMotionProfile.processAccelerate(speeds, fromDistance, mySpeed, myTurnSpeed);
        }
//        else if (myEndPoint)
//        {    // If the robot is closer to the "to" point, do deceleration
//            myMotionProfile.processDecelerate(speeds, toDistance, mySpeed, myTurnSpeed);
//        }
        myMotionProfile.processHeading(speeds, relativeAngleToPosition, myTurnSpeed);
    }


    /**
     * Wraps the able so it is always in the range [-180, 180].
     *
     * @param angle Angle to be wrapped, in radians.
     * @return The wrapped angle, in radians.
     */
    public double angleWrap(double angle)
    {
        if (angle > 0)
        {
            return ((angle + Math.PI) % (Math.PI * 2)) - Math.PI;
        }
        else
        {
            return ((angle - Math.PI) % (Math.PI * -2)) + Math.PI;
        }
    }

    /**
     * Calculates whether or not two points are equal within a margin of error.
     *
     * @param p1     Point 1
     * @param p2     Point 2
     * @param buffer Margin of error.
     * @return True if the point are equal within a margin or error, false otherwise.
     */
    public boolean positionEqualsWithBuffer(Translation2d p1, Translation2d p2, double buffer)
    {
        if (p1.getX() - buffer < p2.getX() && p1.getX() + buffer > p2.getX())
        {
            if (p1.getY() - buffer < p2.getY() && p1.getY() + buffer > p2.getY())
            {
                return true;
            }
        }
        return false;
    }

//    /**
//     * Calculates whether or not two angles are equal within a margin of error.
//     *
//     * @param a1     Angle 1 (in radians).
//     * @param a2     Angle 2 (in radians).
//     * @param buffer Margin of error (in radians)
//     * @return True if the point are equal within a margin or error, false otherwise.
//     */
//    public boolean rotationEqualsWithBuffer(double a1, double a2, double buffer)
//    {
//        if (a1 - buffer < a2 && a1 + buffer > a2)
//        {
//            return true;
//        }
//        return false;
//    }

    @Override
    public void end(boolean interrupted)
    {
        if ( myStopAtEnd)
        {
            myChassisSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return myArrived;
    }
}

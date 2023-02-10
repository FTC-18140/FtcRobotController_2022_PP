package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;

import static java.lang.Math.abs;
import static java.lang.Math.ulp;

public class ArriveLocationCommand extends CommandBase
{
    private final ChassisSubsystem myChassisSubsystem;
    private final DiffOdometrySubsystem myOdometrySubsystem;
    private final MotionProfile myMotionProfile = new MotionProfile(20, 20, 1);
    private Translation2d fromPoint;
    private final Translation2d toPoint;
    private Pose2d myRobotPose;
    private final double myBuffer;
    private final double myHeading;
    private final double mySpeed;
    private final double myTurnSpeed;
    private boolean myArrived = false;
//    private boolean myAligned = false;
    private boolean myEndPoint = false;
    private boolean myTurnOnly = false;
    // Get delta values.
    double fromDeltaX;
    double fromDeltaY;
    double toDeltaX;
    double toDeltaY;
    // Get Distances
    double fromDistance;
    double toDistance;
    double absoluteAngleToPosition;
    double relativeAngleToPosition;

    Telemetry telemetry;

    /**
     * Creates a new ArriveLocationCommand.
     *
     */
    public ArriveLocationCommand(double x, double y, double heading, double speed, double turnSpeed, double endBuffer, boolean lastPoint, ChassisSubsystem chassis, DiffOdometrySubsystem odometry)
    {
        toPoint = new Translation2d(x, y);
        myHeading = Math.toRadians(heading);
        mySpeed = speed;
        myTurnSpeed = turnSpeed;
        myBuffer = endBuffer;
        myEndPoint = lastPoint;
        myChassisSubsystem = chassis;
        myOdometrySubsystem = odometry;
        telemetry = myChassisSubsystem.getTelemetry();
        myTurnOnly = (mySpeed == 0.0);


        // temp
        myMotionProfile.telem = telemetry;

        addRequirements(myChassisSubsystem);

    }

    @Override
    public void initialize()
    {
        fromPoint = new Translation2d(myOdometrySubsystem.getPose().getTranslation().getX(), myOdometrySubsystem.getPose().getTranslation().getY());
        telemetry.addData("Arrive Command Initialized,", toPoint);
    }

    @Override
    public void execute()
    {
        telemetry.addData("executing Arrive Command", toPoint);
        myRobotPose = myOdometrySubsystem.getPose();

        // Check if we have arrived
        if ( myTurnOnly )
        {
            myArrived = rotationEqualsWithBuffer(myRobotPose.getHeading(), myHeading, 2/180.0*Math.PI);
        }
        else
        {
            myArrived = positionEqualsWithBuffer(myRobotPose.getTranslation(), toPoint, myBuffer);
        }

        telemetry.addData("Arrived at toPoint?  ", myArrived);

        // Get delta Translation values.
        fromDeltaX = myRobotPose.getTranslation().getX() - fromPoint.getX();
        fromDeltaY = myRobotPose.getTranslation().getY() - fromPoint.getY();
        toDeltaX = toPoint.getX() - myRobotPose.getTranslation().getX();
        toDeltaY = toPoint.getY() - myRobotPose.getTranslation().getY();
        telemetry.addData("dX2Point, dY2Point: ", "%.3f, %.3f", toDeltaX,toDeltaY);

        // Get Distances between From and To points
        fromDistance = Math.hypot(fromDeltaX, fromDeltaY);
        toDistance = Math.hypot(toDeltaX, toDeltaY);

        // Get angle to To point, both absolute and relative to robot's current heading
        absoluteAngleToPosition = Math.atan2(toDeltaY, toDeltaX);
        relativeAngleToPosition = -angleWrap(absoluteAngleToPosition - myRobotPose.getHeading());

        // The x and y powers need to be swapped and have their signs flipped.
        telemetry.addData("Distance to Point: ", toDistance);
        telemetry.addData("AbsAngle, deg: ", Math.toDegrees(absoluteAngleToPosition));

        // Determine if robot needs to drive and turn to get to the position.
        double[] motorPowers = new double[3];
        if ( myTurnOnly)
        { // no translation speed
            telemetry.addData("RobotHeading: ", Math.toDegrees(myRobotPose.getHeading()));
            telemetry.addData("myHeading: ", Math.toDegrees(myHeading));
            relativeAngleToPosition = -angleWrap(myHeading - myRobotPose.getHeading());
            motorPowers[0] = 0.0;
            motorPowers[2] = Range.clip( -relativeAngleToPosition/Math.PI, -1.0*myTurnSpeed, myTurnSpeed);
        }
        else
        {  // find translation speed
            motorPowers[0] = Range.clip(toDistance, 0.1, mySpeed);
            motorPowers[2] = Range.clip( -relativeAngleToPosition/Math.PI, -1.0*myTurnSpeed, myTurnSpeed);
        }
        motorPowers[1] = 0;
        telemetry.addData("Relative Angle, deg: ", Math.toDegrees(relativeAngleToPosition));


//        telemetry.addData("Power 0, raw: ", motorPowers[0]);
//        telemetry.addData("Power 2, raw: ", motorPowers[2]);

        // Do the motion profiling on the motor powers based on where we are relative to the target
        profileMotorPowers(motorPowers);

        telemetry.addData("Power 0, profiled: ", motorPowers[0]);
        telemetry.addData("Power 2, profiled: ", motorPowers[2]);

        if (myArrived)
        {
            motorPowers[0] = 0;
            motorPowers[1] = 0;
            motorPowers[2] = 0;
        }

        myChassisSubsystem.arcadeDrive(motorPowers[0], motorPowers[2]);
    }

    /**
     * Adjusts the motor speeds based on this path's motion profile.
     *
     * @param speeds       Speeds to be adjusted.
     */
    private void profileMotorPowers(double[] speeds)
    {
        if (fromDistance < toDistance)
        {    // If the robot is closer to the "from" point, do acceleration
            myMotionProfile.processAccelerate(speeds, fromDistance, mySpeed, myTurnSpeed);
        }
        else if (myEndPoint)
        {    // If the robot is closer to the "to" point, do deceleration
            myMotionProfile.processDecelerate(speeds, toDistance, mySpeed, myTurnSpeed);
        }
        myMotionProfile.processHeading(speeds, relativeAngleToPosition, myTurnSpeed);
    }

//    /**
//     * Takes the robot's current position and rotation and calculates the motor powers for the robot to move to the target position.
//     *
//     * @param cx       Robot's current X position.
//     * @param cy       Robot's current Y position.
//     * @param ca       Robot's current rotation (angle).
//     * @param tx       Target X position.
//     * @param ty       Target Y position.
//     * @param toAngle       Target rotation (angle).
//     * @param turnOnly True if the robot should only turn.
//     * @return A double array containing raw motor powers. a[0] is strafe power, a[1] is vertical power and a[2] is turn power.
//     */
//    public double[] One_moveToPosition(double cx, double cy, double ca, double tx, double ty, double toAngle, boolean turnOnly)
//    {
//        double[] rawMotorPowers;
//
//        if (turnOnly)
//        {
//            // If turnOnly is true, only return a turn power.
//            return new double[]{0, 0, angleWrap(ca + toAngle) / Math.PI};
//        }
////
////        double absoluteXToPosition = tx - cx;
////        double absoluteYToPosition = ty - cy;
//
//        double absoluteAngleToPosition = Math.atan2(toDetlaY, toDeltaX);
////        double distanceToPosition = Math.hypot(toDeltaX, toDetlaY);
//
//        double relativeAngleToPosition = angleWrap(absoluteAngleToPosition + ca);
//
//        rawMotorPowers = new double[3];
//
//        // The x and y powers need to be swapped and have their signs flipped.
//        telemetry.addData("Distance to Point: ", toDistance);
//        telemetry.addData("Relative Angle: ", relativeAngleToPosition/Math.PI*180.0);
//        rawMotorPowers[0] = Range.clip(toDistance, -1.0*mySpeed, mySpeed);
//        rawMotorPowers[1] = 0;
//        rawMotorPowers[2] = Range.clip( angleWrap(relativeAngleToPosition)/Math.PI, -1.0*myTurnSpeed, myTurnSpeed);
//
//        return rawMotorPowers;
//    }


//    /**
//     * Normalizes the provided motor speeds to be in the range [-maxTranslate, maxTranslate].
//     *
//     * @param speeds Motor speeds to normalize.
//     */
//    private void Three_normalizeMotorSpeeds(double[] speeds, double maxTranslate, double maxTurn )
//    {
//        double max = Math.max(abs(speeds[0]), abs(speeds[1]));
//
//        if (max > maxTranslate)
//        {
//            speeds[0] *= maxTranslate/max;
//            speeds[1] *= maxTranslate/max;
//        }
//
//        max = Math.abs(speeds[2]);
//        if (max > maxTurn)
//        {
//            speeds[2] *= maxTurn/max;
//        }
////        else if (speeds[2] < -1*maxTurn)
////        {
////            speeds[2] = -1 * maxTurn;
////        }
//    }


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
            return ((angle - Math.PI) % (Math.PI * 2)) + Math.PI;
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

    /**
     * Calculates whether or not two angles are equal within a margin of error.
     *
     * @param a1     Angle 1 (in radians).
     * @param a2     Angle 2 (in radians).
     * @param buffer Margin of error (in radians)
     * @return True if the point are equal within a margin or error, false otherwise.
     */
    public boolean rotationEqualsWithBuffer(double a1, double a2, double buffer)
    {
        if (a1 - buffer < a2 && a1 + buffer > a2)
        {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        myChassisSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return myArrived;
    }
}

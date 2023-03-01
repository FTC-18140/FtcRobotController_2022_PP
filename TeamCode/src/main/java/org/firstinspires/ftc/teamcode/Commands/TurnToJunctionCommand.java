package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

public class TurnToJunctionCommand extends CommandBase
{
    private final ChassisSubsystem myChassisSubsystem;
    private final DiffDriveOdometrySubsystem myOdometrySubsystem;
    private final MotionProfile myMotionProfile;


    private final double toHeadingRad;
    private final double distance;

    private Pose2d myRobotPose;

    private final double myTargetZoneRad;

    private final double myMaxTurnSpeed;
    private final double myMinTurnSpeed;

    private boolean myFinished = false;

    double relativeAngleToPosition;

    Telemetry telemetry;

    /**
     * Creates a new ArriveCommand.
     *
     */
    public TurnToJunctionCommand(double headingDeg, double turnSpeed, double minTurnSpeed, double targetBufferDeg, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {

        toHeadingRad = Math.toRadians(headingDeg);

        myMaxTurnSpeed = turnSpeed;
        myMinTurnSpeed = minTurnSpeed;
        myTargetZoneRad = Math.toRadians(targetBufferDeg);

        myChassisSubsystem = chassis;
        myOdometrySubsystem = odometry;
        telemetry = myChassisSubsystem.getTelemetry();

        myMotionProfile =  new MotionProfile(20, 10, 1, 0.02);

        distance = 20;

        // temp
        myMotionProfile.telem = telemetry;

        addRequirements(myChassisSubsystem, myOdometrySubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        telemetry.addData("Executing Turn Command: ", toHeadingRad);
        // Update the current position of the Robot by getting it from the odometry subsystem.
        myRobotPose = myOdometrySubsystem.getPose();


        // Now that the robot's position is updated. Figure out the angle the robot needs to
        // drive in order to get to the To Point.
        double turnAngle = updateAngles();

        // Determine if robot needs to drive and turn to get to the position.
        double[] motorPowers = new double[3];
        motorPowers[1] = 0; // no strafing


        telemetry.addData("RobotHeading: ", Math.toDegrees(myRobotPose.getHeading()));
        telemetry.addData("myHeading: ", Math.toDegrees(toHeadingRad));
        motorPowers[0] = 0.0;
        motorPowers[2] = Range.clip(-turnAngle, -1.0 * myMaxTurnSpeed, myMaxTurnSpeed);


        // Do the motion profiling on the motor powers based on where we are relative to the target
        profileMotorPowers(motorPowers);

        telemetry.addData("Power 0, profiled: ", motorPowers[0]);
        telemetry.addData("Power 2, profiled: ", motorPowers[2]);

        // Check if we have arrived

        myFinished = myChassisSubsystem.getDistance() < distance;


        telemetry.addData("Arrived at toPoint?  ", myFinished);
        if (myFinished)
        {
            motorPowers[0] = 0;
            motorPowers[1] = 0;
            motorPowers[2] = 0;
        }

        if ((Math.abs(motorPowers[0])+Math.abs(motorPowers[2])) > myMaxTurnSpeed)
        {
            motorPowers[0] = motorPowers[0] * Math.abs(motorPowers[0]/(Math.abs(motorPowers[0])+Math.abs(motorPowers[2])));
        }
        myChassisSubsystem.arcadeDrive(motorPowers[0], motorPowers[2]);
    }


    private double updateAngles()
    {

        // Based on the current heading of the robot, update the value that determines how much
        // the robot needs to turn

        relativeAngleToPosition = -angleWrap(toHeadingRad - myRobotPose.getHeading());

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
        myMotionProfile.processHeading(speeds, relativeAngleToPosition, myMaxTurnSpeed);
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
        return myFinished;
    }
}

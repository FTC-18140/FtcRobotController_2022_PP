package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.PurePursuitUtil;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;

import static java.lang.Math.abs;

public class ArriveLocationCommand extends CommandBase {

    private final ChassisSubsystem myChassisSubsystem;
    private final DiffOdometrySubsystem myOdometrySubsystem;
    private final MotionProfile myMotionProfile = new MotionProfile(10, 10, 10);
    private Translation2d fromPoint;
    private final Translation2d toPoint;
    private Pose2d myRobotPose;
    private final double myBuffer;
    private final double myHeading;
    private final double mySpeed;
    private final double myTurnSpeed;
    private boolean myArrived = false;
    private boolean myAligned = false;
    private boolean myEndPoint = false;
    Telemetry telemetry;

    /**
     * Creates a new ArriveLocationCommand.
     *
     */
    public ArriveLocationCommand(double x, double y, double heading, double speed, double turnSpeed, double endBuffer, boolean lastPoint, ChassisSubsystem chassis, DiffOdometrySubsystem odometry)
    {
        toPoint = new Translation2d(x, y);
        myHeading = heading;
        mySpeed = speed;
        myTurnSpeed = turnSpeed;
        myBuffer = endBuffer;
        myEndPoint = lastPoint;
        myChassisSubsystem = chassis;
        myOdometrySubsystem = odometry;
        telemetry = myChassisSubsystem.getTelemetry();

    }

    @Override
    public void initialize() {
        fromPoint = new Translation2d(myOdometrySubsystem.getPose().getTranslation().getX(), myOdometrySubsystem.getPose().getTranslation().getY());

    }

    @Override
    public void execute()
    {
        myRobotPose = myOdometrySubsystem.getPose();
        myArrived = PurePursuitUtil.positionEqualsWithBuffer(myRobotPose.getTranslation(), toPoint, myBuffer);
        myAligned = PurePursuitUtil.rotationEqualsWithBuffer(myRobotPose.getHeading(), myHeading, 2);

        telemetry.addData("Arrived at toPoint?  ", myArrived);

        double[] motorPowers = new double[]{0, 0, 0};
        motorPowers = PurePursuitUtil.moveToPosition(myRobotPose.getX(), myRobotPose.getY(), myRobotPose.getHeading(), toPoint.getX(), toPoint.getY(), myHeading, false);
        telemetry.addData("motor Powers 0: ", motorPowers[0]);
        telemetry.addData("motor Powers 1: ", motorPowers[1]);
        telemetry.addData("motor Powers 2: ", motorPowers[2]);

        adjustSpeedsWithProfile(motorPowers, myRobotPose.getTranslation(), myRobotPose.getHeading());
        telemetry.addData("motor Powers 0, profiled: ", motorPowers[0]);
        telemetry.addData("motor Powers 1, profiled: ", motorPowers[1]);
        telemetry.addData("motor Powers 2, profiled: ", motorPowers[2]);

        normalizeMotorSpeeds(motorPowers, mySpeed, myTurnSpeed);
        telemetry.addData("motor Powers 0, normalized: ", motorPowers[0]);
        telemetry.addData("motor Powers 1, normalized: ", motorPowers[1]);
        telemetry.addData("motor Powers 2, normalized: ", motorPowers[2]);

        if (myArrived)
        {
            motorPowers[0] = 0;
            motorPowers[1] = 0;
        }
        if (myAligned)
        {
            motorPowers[2] = 0;
        }

        myChassisSubsystem.arcadeDrive(motorPowers[0], motorPowers[2]);

    }

    @Override
    public void end(boolean interrupted) {
        myChassisSubsystem.stop();
    }


    @Override
    public boolean isFinished() {
        return myArrived && myAligned;
    }


    /**
     * Adjusts the motor speeds based on this path's motion profile.
     *
     * @param speeds       Speeds to be adjusted.
     */
    private void adjustSpeedsWithProfile(double[] speeds, Translation2d robotXY, double robotHdg )
    {
        // Get delta values.
        double adx = robotXY.getX() - fromPoint.getX();
        double ady = robotXY.getY() - fromPoint.getY();
        double tdx = toPoint.getX() - robotXY.getX();
        double tdy = toPoint.getY() - robotXY.getY();
        // Get Distances
        double ad = Math.hypot(adx, ady);
        double td = Math.hypot(tdx, tdy);

        if (ad < td)
        {    // If the robot is closer to the "from" point, do acceleration
            myMotionProfile.processAccelerate(speeds, ad, mySpeed, myTurnSpeed);
        }
        else if (myEndPoint)
        {    // If the robot is closer to the "to" point, do deceleration
            myMotionProfile.processDecelerate(speeds, td, mySpeed, myTurnSpeed);
        }
        myMotionProfile.processHeading(speeds, myRobotPose.getHeading() - myHeading, myTurnSpeed);
    }


    /**
     * Normalizes the provided motor speeds to be in the range [-1, 1].
     *
     * @param speeds Motor speeds to normalize.
     */
    private void normalizeMotorSpeeds(double[] speeds, double maxTranslate, double maxTurn )
    {
        double max = Math.max(abs(speeds[0]), abs(speeds[1]));

        if (max > maxTranslate) {
            speeds[0] /= max;
            speeds[1] /= max;
        }
        if (speeds[2] > maxTurn)
            speeds[2] = maxTurn;
        else if (speeds[2] < -1*maxTurn)
            speeds[2] = -1*maxTurn;
    }
}

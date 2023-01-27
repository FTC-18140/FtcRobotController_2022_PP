package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.PathMotionProfile;
import com.arcrobotics.ftclib.purepursuit.PurePursuitUtil;
import com.arcrobotics.ftclib.purepursuit.types.WaypointType;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;

import org.checkerframework.checker.propkey.qual.PropertyKeyBottom;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;

public class ArriveLocationCommand extends CommandBase {

    private final ChassisSubsystem myChassisSubsystem;
    private final DiffOdometrySubsystem myOdometrySubsystem;
    private final MotionProfile motionProfile = new MotionProfile();
    private Translation2d fromPoint;
    private final Translation2d toPoint;
    private Pose2d robotPos;
    private final double myBuffer;
    private final double myHeading;
    private final double mySpeed;
    private final double myTurnSpeed;
    private boolean arrived = false;
    Telemetry telemetry;

    /**
     * Creates a new DriveDistanceCommand.
     *
     * @param speed  The speed at which the robot will drive
     * @param chassis  The drive subsystem on which this command will run
     */
    public ArriveLocationCommand(double x, double y, double heading, double speed, double turnSpeed, double endBuffer, ChassisSubsystem chassis, DiffOdometrySubsystem odometry) {
        toPoint = new Translation2d(x, y);
        myHeading = heading;
        mySpeed = speed;
        myTurnSpeed = turnSpeed;
        myBuffer = endBuffer;
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
        robotPos = myOdometrySubsystem.getPose();
        arrived = PurePursuitUtil.positionEqualsWithBuffer(robotPos.getTranslation(), toPoint, myBuffer);
        telemetry.addData("Arrived at toPoint?  ", arrived);

        double[] motorPowers = new double[]{0, 0, 0};
        motorPowers = PurePursuitUtil.moveToPosition( robotPos.getX(), robotPos.getY(), robotPos.getHeading(), toPoint.getX(), toPoint.getY(), myHeading, false );
        telemetry.addData("motor Powers 0: ", motorPowers[0]);
        telemetry.addData("motor Powers 1: ", motorPowers[1]);
        telemetry.addData("motor Powers 2: ", motorPowers[2]);

        adjustSpeedsWithProfile(motorPowers, robotPos.getTranslation());
        telemetry.addData("motor Powers 0, profiled: ", motorPowers[0]);
        telemetry.addData("motor Powers 1, profiled: ", motorPowers[1]);
        telemetry.addData("motor Powers 2, profiled: ", motorPowers[2]);

        normalizeMotorSpeeds(motorPowers, mySpeed, myTurnSpeed);
        telemetry.addData("motor Powers 0, normalized: ", motorPowers[0]);
        telemetry.addData("motor Powers 1, normalized: ", motorPowers[1]);
        telemetry.addData("motor Powers 2, normalized: ", motorPowers[2]);

        if (arrived)
        {
            motorPowers[0] = 0;
            motorPowers[1] = 0;
        }
        myChassisSubsystem.joystickDrive(motorPowers[0], motorPowers[1], motorPowers[2]);

    }

    @Override
    public void end(boolean interrupted) {
        myChassisSubsystem.stop();
    }


    @Override
    public boolean isFinished() {
        return arrived && PurePursuitUtil.rotationEqualsWithBuffer(robotPos.getHeading(), myHeading, 2);
    }


    /**
     * Adjusts the motor speeds based on this path's motion profile.
     *
     * @param speeds       Speeds to be adjusted.
     */
    private void adjustSpeedsWithProfile(double[] speeds, Translation2d robotPos) {

        // Get delta values.
        double adx = robotPos.getX() - fromPoint.getX();
        double ady = robotPos.getY() - fromPoint.getY();
        double tdx = toPoint.getX() - robotPos.getX();
        double tdy = toPoint.getY() - robotPos.getY();
        double ad = Math.hypot(adx, ady);
        double td = Math.hypot(tdx, tdy);
        if (ad < td)
            // If the intersection is closer to the from point.
            motionProfile.processAccelerate(speeds, ad, mySpeed, myTurnSpeed);
        else
            // If the intersection is closer to the to point.
            motionProfile.processDecelerate(speeds, td, mySpeed, myTurnSpeed);
    }


    /**
     * Normalizes the provided motor speeds to be in the range [-1, 1].
     *
     * @param speeds Motor speeds to normalize.
     */
    private void normalizeMotorSpeeds(double[] speeds, double maxTranslate, double maxTurn ) {
        double max = Math.max(Math.abs(speeds[0]), Math.abs(speeds[1]));

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

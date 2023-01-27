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
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;

public class ArriveLocationCommand extends CommandBase {

    private final ChassisSubsystem myChassisSubsystem;
    private final DiffOdometrySubsystem myOdometrySubsystem;
    private final MotionProfile motionProfile = new MotionProfile();
    private final Translation2d toPoint;
    private Translation2d awayPoint;
    private Pose2d robotPos;
    private final double myBuffer;
    private final double myHeading;
    private final double mySpeed;
    private final double myTurnSpeed;
    private boolean arrived = false;

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

    }

    @Override
    public void initialize() {
      //  awayPoint = new Translation2d(myOdometrySubsystem.getPose().getTranslation().getX(), myOdometrySubsystem.getPose().getTranslation().getY());
          awayPoint = toPoint;
    }

    @Override
    public void execute()
    {
        robotPos = myOdometrySubsystem.getPose();
        arrived = PurePursuitUtil.positionEqualsWithBuffer(robotPos.getTranslation(), toPoint, myBuffer);
        double[] motorPowers = new double[]{0, 0, 0};
        motorPowers = PurePursuitUtil.moveToPosition( robotPos.getX(), robotPos.getY(), robotPos.getHeading(), awayPoint.getX(), awayPoint.getY(), myHeading, false );
        myChassisSubsystem.getTelemetry().addData("motor Powers 0", motorPowers[0]);
        myChassisSubsystem.getTelemetry().addData("motor Powers 1", motorPowers[1]);
        myChassisSubsystem.getTelemetry().addData("motor Powers 2", motorPowers[2]);
        adjustSpeedsWithProfile(motorPowers, robotPos.getTranslation());
        normalizeMotorSpeeds(motorPowers);
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
        double adx = robotPos.getX() - awayPoint.getX();
        double ady = robotPos.getY() - awayPoint.getY();
        double tdx = toPoint.getX() - robotPos.getX();
        double tdy = toPoint.getY() - robotPos.getY();
        double ad = Math.hypot(adx, ady);
        double td = Math.hypot(tdx, tdy);
        if (ad < td)
            // If the intersection is closer to the away point.
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
    private void normalizeMotorSpeeds(double[] speeds) {
        double max = Math.max(Math.abs(speeds[0]), Math.abs(speeds[1]));
        if (max > 1) {
            speeds[0] /= max;
            speeds[1] /= max;
        }
        if (speeds[2] > 1)
            speeds[2] = 1;
        else if (speeds[2] < -1)
            speeds[2] = -1;
    }
}

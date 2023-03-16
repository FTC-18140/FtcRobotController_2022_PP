package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandOpModes.AutoEpic;
import org.firstinspires.ftc.teamcode.CommandOpModes.OdometryTesting;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

public abstract class DriveCommandBase extends CommandBase
{
    protected final ChassisSubsystem myChassisSubsystem;
    protected final DiffDriveOdometrySubsystem myOdometrySubsystem;
    protected final MotionProfile myMotionProfile;

    protected Translation2d fromPoint;
    protected Translation2d toPoint;

    protected Pose2d myRobotPose;

    protected final double myEndZoneCM;
    protected final double mySpeed;
    protected final double myTurnSpeed;

    protected boolean myArrived = false;

    protected boolean myStopAtEnd;
    protected final boolean myBackwards;

    // Delta values.
    protected double toDeltaX;
    protected double toDeltaY;

    // Distances
    protected double fromDistance;
    protected double toDistance;
    protected double absoluteAngleToPosition;
    protected double relativeAngleToPosition;

    protected Telemetry telemetry;
    protected boolean firstExecute = true;

    /**
     * Creates a new ArriveCommand.
     *
     */
    public DriveCommandBase(double x, double y, double speed, double turnSpeed, double endZoneCM, boolean stopAtEnd, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {
        toPoint = new Translation2d(x, y);
        mySpeed = speed;
        myBackwards = (Math.signum(speed) < 0);
        myTurnSpeed = turnSpeed;
        myEndZoneCM = endZoneCM;
        myChassisSubsystem = chassis;
        myOdometrySubsystem = odometry;
        telemetry = myChassisSubsystem.getTelemetry();
        myStopAtEnd = stopAtEnd;

        myMotionProfile = new MotionProfile(20, myEndZoneCM, 1, 0.005);
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
        if (firstExecute)
        {
            fromPoint = new Translation2d(myOdometrySubsystem.getPose().getTranslation().getX(), myOdometrySubsystem.getPose().getTranslation().getY());
            firstExecute = false;
        }

        // Get new position data from the odometry subsystem and update the robot's location and
        // distance/angles relative to the From Point and the To Point.
        double driveDistance = updatePositions();
        telemetry.addData("Distance to Point: ", driveDistance);

        // Now that the robot's position is updated. Figure out the angle the robot needs to
        // drive in order to get to the To Point.
        double turnAngle = updateAngles();

        // Set the speeds to move
        double[] motorPowers = new double[3];
        motorPowers[0] = mySpeed;
        motorPowers[1] = 0; // no strafing
        motorPowers[2] = Math.signum(-turnAngle) * myTurnSpeed;
//        motorPowers[2] = Range.clip(-turnAngle, -1.0 * myTurnSpeed, myTurnSpeed);

        // This min turn speed calculation is meant scale the minimum allowed turning speed.  Whe
        // the angle to turn is large, the min turn speed can be large.  When the angle is
        // small, the min turn speed should also be small.
        myMotionProfile.setMinTurnSpeed(0.03 * Math.abs(turnAngle)/Math.toRadians(10));

        // Do the motion profiling on the motor speeds based on where we are relative to the target
        profileMotorPowers(motorPowers);

        telemetry.addData("Power 0, profiled: ", motorPowers[0]);
        telemetry.addData("Power 2, profiled: ", motorPowers[2]);
//        telemetry.addData("MinTurnSpeed: ", 0.03 * Math.abs(turnAngle)/Math.toRadians(10));

        // Check if we have arrived
        // arriving on a normal move is based on closeness to the toPoint
        myArrived = AutoUtils.positionEqualsWithBuffer(myRobotPose.getTranslation(), toPoint, myEndZoneCM);

//        telemetry.addData("Arrived at toPoint?  ", myArrived);

        if (AutoEpic.doLogging)
        {
            AutoEpic.logger.addField(myChassisSubsystem.getLeftEncoderDistance());
            AutoEpic.logger.addField(myChassisSubsystem.getRightEncoderDistance());
            AutoEpic.logger.addField(myOdometrySubsystem.getPose().getX());
            AutoEpic.logger.addField(myOdometrySubsystem.getPose().getY());
            AutoEpic.logger.addField(myOdometrySubsystem.getPose().getHeading());
            AutoEpic.logger.addField(motorPowers[0]);
            AutoEpic.logger.addField(motorPowers[2]);
            AutoEpic.logger.newLine();
        }


        myChassisSubsystem.arcadeDrive(motorPowers[0], motorPowers[2]);
    }

    private double updatePositions()
    {
        // Update the current position of the Robot by getting it from the odometry subsystem.
        myRobotPose = myOdometrySubsystem.getPose();

        // Update delta Translation values so that we can update how far the robot is from the
        // From Point and the To Point
        double fromDeltaX = myRobotPose.getTranslation().getX() - fromPoint.getX();
        double fromDeltaY = myRobotPose.getTranslation().getY() - fromPoint.getY();

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

        relativeAngleToPosition = -AutoUtils.angleWrap(absoluteAngleToPosition - myRobotPose.getHeading());

        if (myBackwards)
        {
            relativeAngleToPosition = AutoUtils.angleWrap(relativeAngleToPosition - Math.PI);
        }

        telemetry.addData("Relative Angle, deg: ", Math.toDegrees(relativeAngleToPosition));
        return relativeAngleToPosition;
    }

    /**
     * Adjusts the motor speeds based on this path's motion profile.
     * Subclasses must implement this.
     *
     * @param speeds       Speeds to be adjusted.
     */
    public abstract void profileMotorPowers(double[] speeds);

    @Override
    public void end(boolean interrupted) {
        if (myStopAtEnd)
        {
            myChassisSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return myArrived;
    }
}

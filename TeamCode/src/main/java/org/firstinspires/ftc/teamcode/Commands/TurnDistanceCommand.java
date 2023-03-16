package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandOpModes.AutoEpic;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

public class TurnDistanceCommand extends CommandBase
{
    private final ChassisSubsystem myChassisSubsystem;
    private final DiffDriveOdometrySubsystem myOdometrySubsystem;
    private final MotionProfile myMotionProfile;


    private final double turnRad;
    private double myInitialAngleRad;

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
    public TurnDistanceCommand(double turnDeg, double turnSpeed, double minTurnSpeed, double headingDecelZoneDeg, double targetBufferDeg, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {


        turnRad = Math.toRadians(turnDeg);

        myMaxTurnSpeed = turnSpeed;
        myMinTurnSpeed = minTurnSpeed;
        myTargetZoneRad = Math.toRadians(targetBufferDeg);

        myChassisSubsystem = chassis;
        myOdometrySubsystem = odometry;
        telemetry = myChassisSubsystem.getTelemetry();

        myMotionProfile =  new MotionProfile(20, 10, headingDecelZoneDeg, 0.02);

        // temp
        myMotionProfile.telem = telemetry;

        addRequirements(myChassisSubsystem, myOdometrySubsystem);
    }

    @Override
    public void initialize()
    {
        myChassisSubsystem.setZeroBehavior(Motor.ZeroPowerBehavior.BRAKE);
        myInitialAngleRad = myOdometrySubsystem.getPose().getHeading();
    }

    @Override
    public void execute()
    {
        telemetry.addData("Executing Turn Command: ", turnRad);
        // Update the current position of the Robot by getting it from the odometry subsystem.
        myRobotPose = myOdometrySubsystem.getPose();

        // Now that the robot's position is updated. Figure out the angle the robot needs to
        // drive in order to get to the To Point.
       // double turnAngle = updateAngles();

        // Determine if robot needs to drive and turn to get to the position.
        double[] motorPowers = new double[3];
        motorPowers[1] = 0; // no strafing

//        telemetry.addData("RobotHeading: ", Math.toDegrees(myRobotPose.getHeading()));
//        telemetry.addData("myHeading: ", Math.toDegrees(turnRad));
        motorPowers[0] = 0.0;
        if (turnRad > 0) {
            motorPowers[2] = myMaxTurnSpeed;
        }else {
            motorPowers[2] = -myMaxTurnSpeed;
            }

        // Do the motion profiling on the motor powers based on where we are relative to the target
       // profileMotorPowers(motorPowers);

//        telemetry.addData("Power 0, profiled: ", motorPowers[0]);
//        telemetry.addData("Power 2, profiled: ", motorPowers[2]);

        // Check if we have arrived

   //     myFinished = rotationEqualsWithBuffer(myRobotPose.getHeading(), turnRad, myTargetZoneRad);

    //    telemetry.addData("Arrived at target Heading?  ", myFinished);

        AutoEpic.logger.addField(myChassisSubsystem.getLeftEncoderDistance());
        AutoEpic.logger.addField(myChassisSubsystem.getRightEncoderDistance());
        AutoEpic.logger.addField(myOdometrySubsystem.getPose().getX());
        AutoEpic.logger.addField(myOdometrySubsystem.getPose().getY());
        AutoEpic.logger.addField(myOdometrySubsystem.getPose().getHeading());
        AutoEpic.logger.addField(motorPowers[0]);
        AutoEpic.logger.addField(motorPowers[2]);
        AutoEpic.logger.newLine();

        myChassisSubsystem.arcadeDrive(motorPowers[0], motorPowers[2]);
    }

    private double updateAngles()
    {
        // Based on the current heading of the robot, update the value that determines how much
        // the robot needs to turn

        relativeAngleToPosition = -AutoUtils.angleWrap(turnRad - myRobotPose.getHeading());

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
        myMotionProfile.processTurn(speeds, relativeAngleToPosition, myMaxTurnSpeed);
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
        return Math.abs(myRobotPose.getHeading() - myInitialAngleRad) >= Math.abs(turnRad);
    }
}

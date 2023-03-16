package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffDriveOdometrySubsystem;

import java.util.function.DoubleSupplier;
@Config
public class TurnToJunctionCommand extends CommandBase
{
    private final ChassisSubsystem myChassisSubsystem;
    private final DiffDriveOdometrySubsystem myOdometrySubsystem;
    private final MotionProfile myMotionProfile;
    private DoubleSupplier distance;

    private final boolean myClockwise;
    private Pose2d myRobotPose;

    public double myDistanceThreshold = 18;
    private final double myMaxTurnSpeed;
    private boolean myFinished = false;

    Telemetry telemetry;

    /**
     * Creates a new ArriveCommand.
     *
     */
    public TurnToJunctionCommand(boolean clockwise, double turnSpeed, DoubleSupplier distanceSupplier, double distThreshold, ChassisSubsystem chassis, DiffDriveOdometrySubsystem odometry)
    {
        myClockwise = clockwise;

        myMaxTurnSpeed = turnSpeed;
        distance = distanceSupplier;
        myDistanceThreshold = distThreshold;

        myChassisSubsystem = chassis;
        myOdometrySubsystem = odometry;
        telemetry = myChassisSubsystem.getTelemetry();

        myMotionProfile =  new MotionProfile(20, 10, 1, 0.02);

        // temp
        myMotionProfile.telem = telemetry;

        addRequirements(myChassisSubsystem, myOdometrySubsystem);
    }

    @Override
    public void initialize()
    {
        telemetry.addData("Initializing Turn to Junction Command: ", myDistanceThreshold);
        myChassisSubsystem.setZeroBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void execute()
    {
        telemetry.addData("Executing Turn to Junction Command: ", myClockwise);
        // Update the current position of the Robot by getting it from the odometry subsystem.
        myRobotPose = myOdometrySubsystem.getPose();

        // Determine if robot needs to drive and turn to get to the position.
        double[] motorPowers = new double[3];
        motorPowers[0] = 0.0; // no translation
        motorPowers[1] = 0.0; // no strafing

        if (myClockwise)
        {
            motorPowers[2] = myMaxTurnSpeed;
        }
        else
        {
            motorPowers[2] = -myMaxTurnSpeed;
        }

        // Do the motion profiling on the motor powers based on where we are relative to the target
//        profileMotorPowers(motorPowers);

        telemetry.addData("Power 0, profiled: ", motorPowers[0]);
        telemetry.addData("Power 2, profiled: ", motorPowers[2]);

        // Check if we have arrived
        myFinished = distance.getAsDouble() < myDistanceThreshold;   ///   Am I seeing the pole???

        telemetry.addData("See Junction?  ", myFinished);

        myChassisSubsystem.arcadeDrive(motorPowers[0], motorPowers[2]);
    }

//    /**
//     * Adjusts the motor speeds based on this path's motion profile.
//     *
//     * @param speeds       Speeds to be adjusted.
//     */
//    private void profileMotorPowers(double[] speeds)
//    {
//        myMotionProfile.processHeading(speeds, relativeAngleToPosition, myMaxTurnSpeed);
//    }


    @Override
    public void end(boolean interrupted) {
        myChassisSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return myFinished;
    }
}

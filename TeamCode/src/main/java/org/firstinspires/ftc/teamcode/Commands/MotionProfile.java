package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.purepursuit.PathMotionProfile;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

public class MotionProfile extends PathMotionProfile
{
    private final double accelBuffer; // cm from target
    private final double decelBuffer; // cm to target
    private final double hdgBuffer; // degrees from target

    public MotionProfile(double accelerationBuffer, double decelerationBuffer, double headingBuffer)
    {
        accelBuffer = accelerationBuffer;
        decelBuffer = decelerationBuffer;
        hdgBuffer = headingBuffer;
    }

    public MotionProfile()
    {
        this( 10, 10, 10 );
    }

    @Override
    public void decelerate(double[] motorSpeeds, double distanceToTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed)
    {
        if (distanceToTarget < decelBuffer) {
            motorSpeeds[0] = rangeWithLimit(motorSpeeds[0], configuredMovementSpeed, distanceToTarget, decelBuffer);
            motorSpeeds[1] = rangeWithLimit(motorSpeeds[1], configuredMovementSpeed, distanceToTarget, decelBuffer);
        }
    }

    @Override
    public void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
        if (distanceFromTarget < accelBuffer) {
            motorSpeeds[0] = rangeWithLimit(motorSpeeds[0], configuredMovementSpeed, distanceFromTarget, accelBuffer);
            motorSpeeds[1] = rangeWithLimit(motorSpeeds[1], configuredMovementSpeed, distanceFromTarget, accelBuffer);
        }

    }

    private double rangeWithLimit( double currentSpeed, double maxSpeed, double distance, double bufferDistance )
    {
        double sign = Math.signum(currentSpeed);
        return Range.clip(currentSpeed * distance/bufferDistance + sign*0.1, -1.0*Math.abs(maxSpeed), Math.abs(maxSpeed));
    }

    public void processHeading(double[] motorspeeds, double angleFromTarget, double configuredTurnSpeed)
    {
        // Do the profiling of the turning.
        double sign = Math.signum(motorspeeds[2]);
        motorspeeds[2] = Range.clip( motorspeeds[2] * abs(angleFromTarget)/hdgBuffer + sign*0.05, -1.0*Math.abs(motorspeeds[2]), Math.abs(motorspeeds[2]));
    }
}

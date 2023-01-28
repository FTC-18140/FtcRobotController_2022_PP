package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.purepursuit.PathMotionProfile;

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
            motorSpeeds[0] =  motorSpeeds[0] * distanceToTarget/decelBuffer + 0.1;
            motorSpeeds[1] =  motorSpeeds[1] * distanceToTarget/decelBuffer + 0.1;
        }
    }

    @Override
    public void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
        if (distanceFromTarget < accelBuffer) {
            motorSpeeds[0] =  configuredMovementSpeed * distanceFromTarget/accelBuffer + 0.1;
            motorSpeeds[1] =  configuredMovementSpeed * distanceFromTarget/accelBuffer + 0.1;
        }
    }

    public void processHeading(double[] motorspeeds, double angleFromTarget, double configuredTurnSpeed)
    {
        // Do the profiling of the turning.
        motorspeeds[2] = motorspeeds[2] * abs(angleFromTarget)/hdgBuffer + 0.05;
    }

}

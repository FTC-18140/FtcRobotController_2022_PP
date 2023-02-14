package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.purepursuit.PathMotionProfile;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;
import static java.lang.Math.max;

public class MotionProfile extends PathMotionProfile
{
    private final double accelBuffer; // cm from target
    private final double decelBuffer; // cm to target
    private final double hdgBuffer; // degrees from target
    public Telemetry telem;

    public MotionProfile(double accelerationBuffer, double decelerationBuffer, double headingBuffer)
    {
        accelBuffer = accelerationBuffer;
        decelBuffer = decelerationBuffer;
        hdgBuffer = headingBuffer;
    }

    public MotionProfile()
    {
        this( 50, 40, 1 );
    }

    @Override
    public void decelerate(double[] motorSpeeds, double distanceToTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed)
    {
        if (distanceToTarget < decelBuffer) {
            motorSpeeds[0] = shapeWithLimit(motorSpeeds[0], configuredMovementSpeed, distanceToTarget, decelBuffer, 0.1, 0.5);
            motorSpeeds[1] = shapeWithLimit(motorSpeeds[1], configuredMovementSpeed, distanceToTarget, decelBuffer, 0.1, 0.5);
        }
    }

    @Override
    public void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
        if (distanceFromTarget < accelBuffer) {
            motorSpeeds[0] = shapeWithLimit(motorSpeeds[0], configuredMovementSpeed, distanceFromTarget, accelBuffer, 0.2, 1);
            motorSpeeds[1] = shapeWithLimit(motorSpeeds[1], configuredMovementSpeed, distanceFromTarget, accelBuffer, 0.2, 1);
        }

    }

    private double shapeWithLimit(double currentSpeed, double maxSpeed, double distance, double bufferDistance, double minSpeed, double order)
    {
        double sign = Math.signum(currentSpeed);
        //telem.addData("Sign: ", sign);
        double shapedValue = Math.abs(currentSpeed)*Math.pow(Math.abs(distance/bufferDistance), order);
        //telem.addData("Pre shaped value: ", shapedValue);
        return sign*Range.clip(shapedValue, minSpeed, Math.abs(maxSpeed));
    }

    public void processHeading(double[] motorspeeds, double angleFromTarget, double maxTurnSpeed)
    {
        // Do the profiling of the turning.
        //telem.addData("value to shape: ", "%.3f, %.3f, %.3f", motorspeeds[2], angleFromTarget, maxTurnSpeed);
        double shapedValue = shapeWithLimit(motorspeeds[2], maxTurnSpeed, angleFromTarget, hdgBuffer/180.0*Math.PI, 0.1, 1);
        //telem.addData("shaped Value: ", shapedValue);
        motorspeeds[2] = shapedValue;
    }
}

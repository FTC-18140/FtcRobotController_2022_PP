package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.purepursuit.PathMotionProfile;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;
import static java.lang.Math.max;
@Config
public class MotionProfile extends PathMotionProfile
{
    private final double accelBufferCM; // cm from target
    private final double decelBufferCM; // cm to target
    private final double hdgBufferDeg; // degrees from target

    public void setMinTurnSpeed(double minTurnSpeed)
    {
        this.minTurnSpeed = minTurnSpeed;
    }

    private double minTurnSpeed;
    public Telemetry telem;

    public MotionProfile(double accelerationBufferCM, double decelerationBufferCM, double headingBufferDeg, double minTurnSpd)
    {
        accelBufferCM = accelerationBufferCM;
        decelBufferCM = decelerationBufferCM;
        hdgBufferDeg = headingBufferDeg;
        minTurnSpeed = minTurnSpd;
    }

    public MotionProfile()
    {
        this(50, 40, 1, 0.02);
    }

    @Override
    public void decelerate(double[] motorSpeeds, double distanceToTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed)
    {
        if (distanceToTarget < decelBufferCM) {
            motorSpeeds[0] = shapeWithLimit(motorSpeeds[0], configuredMovementSpeed, distanceToTarget,
                                            decelBufferCM, 0.15, 1);
            motorSpeeds[1] = shapeWithLimit(motorSpeeds[1], configuredMovementSpeed, distanceToTarget,
                                            decelBufferCM, 0.15, 1);
        }
    }

    @Override
    public void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
        if (distanceFromTarget < accelBufferCM) {
            motorSpeeds[0] = shapeWithLimit(motorSpeeds[0], configuredMovementSpeed, distanceFromTarget,
                                            accelBufferCM, 0.125, 1);
            motorSpeeds[1] = shapeWithLimit(motorSpeeds[1], configuredMovementSpeed, distanceFromTarget,
                                            accelBufferCM, 0.125, 1);
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

    public static double headingOrder = 0.5;

    public void processHeading(double[] motorspeeds, double angleFromTarget, double maxTurnSpeed)
    {
        // Do the profiling of the turning.
        telem.addData("value to shape: ", "%.3f, %.3f, %.3f", motorspeeds[2], angleFromTarget, maxTurnSpeed);
        double shapedValue = shapeWithLimit(motorspeeds[2], maxTurnSpeed, angleFromTarget, Math.toRadians(
                hdgBufferDeg), minTurnSpeed, headingOrder);
        telem.addData("shaped Value: ", shapedValue);
        motorspeeds[2] = shapedValue;
    }
}

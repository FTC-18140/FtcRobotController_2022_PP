package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.purepursuit.PathMotionProfile;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotionProfile extends PathMotionProfile
{
    private double accelBufferCM; // cm from target
    private double decelBufferCM; // cm to target
    private final double hdgBufferDeg; // degrees from target

    private double minTurnSpeed;
    public Telemetry telem;

    public static double headingOrder = 2;
    public static double headingGain = 0.1;
    public static double accelOrder = 1;
    public static double decelOrder = 8;
    public static double minAccelSpeed = 0.2;
    public static double minDecelSpeed = 0.1;
    public static double turnGain = 0.8;


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
                                            decelBufferCM, minDecelSpeed, decelOrder);
            motorSpeeds[1] = shapeWithLimit(motorSpeeds[1], configuredMovementSpeed, distanceToTarget,
                                            decelBufferCM, minDecelSpeed, decelOrder);
        }
    }

    @Override
    public void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
        if (distanceFromTarget < accelBufferCM) {
            motorSpeeds[0] = shapeWithLimit(motorSpeeds[0], configuredMovementSpeed, distanceFromTarget,
                                            accelBufferCM, minAccelSpeed, accelOrder);
            motorSpeeds[1] = shapeWithLimit(motorSpeeds[1], configuredMovementSpeed, distanceFromTarget,
                                            accelBufferCM, minAccelSpeed, accelOrder);
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

    public void processHeading(double[] motorSpeeds, double angleFromTarget, double maxTurnSpeed)
    {
        // Do the profiling of the turning.
//        telem.addData("value to shape: ", "%.3f, %.3f, %.3f", motorSpeeds[2], angleFromTarget, maxTurnSpeed);
        double shapedValue =  headingGain * shapeWithLimit(motorSpeeds[2], maxTurnSpeed, angleFromTarget, Math.toRadians(hdgBufferDeg), minTurnSpeed, headingOrder);
//        telem.addData("shaped Value: ", shapedValue);
        motorSpeeds[2] = shapedValue;
    }

    public void processTurn(double[] motorSpeeds, double angleFromTarget, double maxTurnSpeed)
    {
        // Do the profiling of the turning.
//        telem.addData("value to shape: ", "%.3f, %.3f, %.3f", motorSpeeds[2], angleFromTarget, maxTurnSpeed);
        double shapedValue =  turnGain * shapeWithLimit(motorSpeeds[2], maxTurnSpeed, angleFromTarget, Math.toRadians(hdgBufferDeg), minTurnSpeed, headingOrder);
//        telem.addData("shaped Value: ", shapedValue);
        motorSpeeds[2] = shapedValue;
    }

    public void setMinTurnSpeed(double minTurnSpeed)
    {
        this.minTurnSpeed = minTurnSpeed;
    }
    public void setDecelBufferCM( double bufferCM ) { this.decelBufferCM = bufferCM; }
    public void setAccelBufferCM( double bufferCM ) { this.accelBufferCM = bufferCM; }

}

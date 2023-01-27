package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.purepursuit.PathMotionProfile;

@Config
public class MotionProfile extends PathMotionProfile
{
    public static double accelBuffer = 15; // cm from target
    public static double decelBuffer = 15; // cm to target

    @Override
    public void decelerate(double[] motorSpeeds, double distanceToTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed)
    {
        if (distanceToTarget < decelBuffer) {
            motorSpeeds[0] =  motorSpeeds[0] * distanceToTarget/decelBuffer + 0.1;
            motorSpeeds[1] =  motorSpeeds[1] * distanceToTarget/decelBuffer + 0.1;
            motorSpeeds[2] *= motorSpeeds[2] * distanceToTarget/decelBuffer + 0.05;
        }
    }

    @Override
    public void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
        if (distanceFromTarget < accelBuffer) {
            motorSpeeds[0] =  configuredMovementSpeed * distanceFromTarget/accelBuffer;
            motorSpeeds[1] =  configuredMovementSpeed * distanceFromTarget/accelBuffer;
            motorSpeeds[2] *= configuredTurnSpeed * distanceFromTarget/accelBuffer;
        }
    }

}

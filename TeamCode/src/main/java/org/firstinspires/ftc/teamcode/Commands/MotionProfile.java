package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.purepursuit.PathMotionProfile;

@Config
public class MotionProfile extends PathMotionProfile
{
    public static double accelBuffer = 0.15;
    public static double decelBuffer = 0.15;

    @Override
    public void decelerate(double[] motorSpeeds, double distanceToTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed)
    {
        if (distanceToTarget < decelBuffer) {
            motorSpeeds[0] *= configuredMovementSpeed * ((distanceToTarget * 10) + 0.1);
            motorSpeeds[1] *= configuredMovementSpeed * ((distanceToTarget * 10) + 0.1);
            motorSpeeds[2] *= configuredTurnSpeed;
        } else {
            motorSpeeds[0] *= configuredMovementSpeed;
            motorSpeeds[1] *= configuredMovementSpeed;
            motorSpeeds[2] *= configuredTurnSpeed;
        }
    }

    @Override
    public void accelerate(double[] motorSpeeds, double distanceFromTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
        if (distanceFromTarget < accelBuffer) {
            motorSpeeds[0] *= configuredMovementSpeed * ((distanceFromTarget * 10) + 0.1);
            motorSpeeds[1] *= configuredMovementSpeed * ((distanceFromTarget * 10) + 0.1);
            motorSpeeds[2] *= configuredTurnSpeed;
        } else {
            motorSpeeds[0] *= configuredMovementSpeed;
            motorSpeeds[1] *= configuredMovementSpeed;
            motorSpeeds[2] *= configuredTurnSpeed;
        }
    }
}

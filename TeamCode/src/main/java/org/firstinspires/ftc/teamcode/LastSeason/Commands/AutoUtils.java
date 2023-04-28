package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.arcrobotics.ftclib.geometry.Translation2d;
public class AutoUtils
{
    /**
     * Calculates whether or not two points are equal within a margin of error.
     *
     * @param p1     Point 1
     * @param p2     Point 2
     * @param buffer Margin of error.
     * @return True if the point are equal within a margin or error, false otherwise.
     */
    public static boolean positionEqualsWithBuffer(Translation2d p1, Translation2d p2, double buffer)
    {
        if (p1.getX() - buffer < p2.getX() && p1.getX() + buffer > p2.getX())
        {
            if (p1.getY() - buffer < p2.getY() && p1.getY() + buffer > p2.getY())
            {
                return true;
            }
        }
        return false;
    }

    /**
     * Wraps the able so it is always in the range [-180, 180].
     *
     * @param angle Angle to be wrapped, in radians.
     * @return The wrapped angle, in radians.
     */
    public static double angleWrap(double angle)
    {
        if (angle > 0)
        {
            return ((angle + Math.PI) % (Math.PI * 2)) - Math.PI;
        }
        else
        {
            return ((angle - Math.PI) % (Math.PI * -2)) + Math.PI;
        }
    }
}

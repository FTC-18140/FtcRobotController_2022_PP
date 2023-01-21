package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase
{

    Motor leftMotor;
    Motor rightMotor;
    Motor.Encoder leftEncoder;
    Motor.Encoder rightEncoder;
    MotorGroup motors;

    double leftSlidePosition;
    double rightSlidePosition;

    Telemetry telemetry;

    // Lift parameters
    final private double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    final private double DRIVE_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    final private double SPOOL_DIAMETER_CM = 3.5;  // slide spool is 35mm in diameter
    final private double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (SPOOL_DIAMETER_CM * Math.PI);

    public LiftSubsystem(Motor left, Motor right, Telemetry telem)
    {
        leftMotor = left;
        rightMotor = right;
        telemetry = telem;
        motors = new MotorGroup(left, right);
    }

    public LiftSubsystem(HardwareMap hwMap, String left, String right, Telemetry telem)
    {
        this(new Motor(hwMap, left),
             new Motor(hwMap, right),
             telem);
    }


    /**
     * Makes the lift go up at the power level specified.  This method handles the sign needed
     * for the motor to turn the correct direction.
     *
     * @param power
     */
    public void liftUp(double power){
        // Up power is positive.  Make sure it's positive.
        power = Math.abs(power);

        if (motors != null)
        {
            if ((leftSlidePosition + rightSlidePosition / 2.0)  >= 51)
            {
                motors.stopMotor();
            }
            else if ((leftSlidePosition + rightSlidePosition / 2.0)  > 45)
            {
                motors.set(0.15);
            }
            else
            {
                motors.set(power);
            }
        }

    }

    /**
     * Makes the lift go down at the power level specified.  This method handles the sign needed
     * for the motor to turn the correct direction.
     *
     * @param power
     */
    public void liftDown(double power) {
        // Down power is negative.  Make sure it's negative.
        power = -1.0 * Math.abs(power);

        if ( motors != null)
        {
            if ((leftSlidePosition + rightSlidePosition / 2.0) <= 0)
            {
                motors.stopMotor();
            }
            else if ((leftSlidePosition + rightSlidePosition / 2.0)  < 8)
            {
                 motors.set(-0.15);
            }
            else
            {
                motors.set(power);// -0.5
            }
        }
    }

    public void liftStop() {
        motors.stopMotor();
    }

    public double getAverageEncoderDistance() {
        return (leftSlidePosition + rightSlidePosition) / 2.0;
    }

    public void update() {
        leftSlidePosition = leftEncoder.getDistance();
        rightSlidePosition = rightEncoder.getDistance();
    }

    @Override
    public void periodic()
    {
        super.periodic();
        update();
    }
}

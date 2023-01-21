package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class FTClib_ThunderBot
{
    public ChassisSubsystem myChassis;
    public DiffOdometrySubsystem myOdometry;
    public ArmSubsystem myArmstrong;
    public ClawSubsystem myClaw;

    public void init( HardwareMap hMap, Telemetry telem )
    {
        myChassis = new ChassisSubsystem(hMap,"leftFront", "rightFront", "leftRear", "rightRear", 96.0/25.4, telem);
        myOdometry = new DiffOdometrySubsystem( myChassis::getLeftEncoderDistance, myChassis::getRightEncoderDistance, 15.0 );
        myArmstrong = new ArmSubsystem( hMap, "lelbow", "relbow", "twist", "wrist", telem);
        myClaw = new ClawSubsystem( hMap, "claw", telem);
    }

    /**
     * This code go's through the math behind the mecanum wheel drive.  Given the joystick values,
     * it will calculate the motor commands needed for the mecanum drive.
     *
     * @param foward    - Any forward motion including backwards
     * @param right     - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double foward, double right, double clockwise)
    {
        myChassis.joystickDrive( foward, right, clockwise);
    }

    public void stop()
    {
        myChassis.stop();
    }

}

package org.firstinspires.ftc.teamcode.CommandOpModes;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.TurboDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DiffOdometrySubsystem;

@TeleOp(name = "FTCLib TeleOp")
public class FTCLIb_Teleop extends TBDOpModeBase
{
    GamepadEx pad1 = null;
    GamepadEx pad2 = null;
    ChassisSubsystem chassis = null;
    DiffOdometrySubsystem odometry = null;

    @Override
    public void init()
    {
        try
        {
            pad1 = new GamepadEx( gamepad1 );
            pad2 = new GamepadEx( gamepad2 );

            chassis = new ChassisSubsystem(hardwareMap, telemetry);
            register( chassis );

            DefaultDriveCommand driveChassis = new DefaultDriveCommand( chassis, pad1::getLeftY, pad1::getLeftX, pad1::getRightX);
            register(chassis);
            chassis.setDefaultCommand(driveChassis);

            GamepadButton buttonA =  pad1.getGamepadButton(GamepadKeys.Button.A);
            buttonA.whenPressed( new TurboDriveCommand( chassis, pad1::getLeftY, pad1::getLeftX, pad1::getRightX) );


            odometry = new DiffOdometrySubsystem( chassis::getLeftEncoderDistance,
                                                  chassis::getRightEncoderDistance,
                                                  telemetry );
            register( odometry );
        }
        catch (Exception e)
        {
            telemetry.addData("Something did not initialize properly.", 0);
            telemetry.addData("Ugh: ", "%s, %s, %s", e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3]);
        }
    }
    @Override
    public void init_loop()
    {
        if (odometry != null) { odometry.update(); }
        telemetry.addData("Init Loop is running... ", 1);
    }

    @Override
    public void start()
    {

    }

    @Override
    public void stop()
    {
        super.stop();
        chassis.stop();
    }

}

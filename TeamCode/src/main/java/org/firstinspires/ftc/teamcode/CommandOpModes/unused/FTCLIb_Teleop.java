package org.firstinspires.ftc.teamcode.CommandOpModes.unused;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandOpModes.TBDOpModeBase;
import org.firstinspires.ftc.teamcode.Commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.Commands.DefaultLiftCommand;
import org.firstinspires.ftc.teamcode.Commands.TurboDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

@TeleOp(name = "FTCLib TeleOp", group = "Teleop")
@Disabled
public class FTCLIb_Teleop extends TBDOpModeBase
{
    GamepadEx pad1 = null;
    GamepadEx pad2 = null;
    ChassisSubsystem chassis = null;
    LiftSubsystem lift = null;

    @Override
    public void init()
    {
        try
        {
            pad1 = new GamepadEx( gamepad1 );
            pad2 = new GamepadEx( gamepad2 );

            chassis = new ChassisSubsystem(hardwareMap, telemetry);
            register( chassis );

            DefaultDriveCommand driveChassis = new DefaultDriveCommand(chassis, pad1::getLeftY, pad1::getLeftX, pad1::getRightX);
            register(chassis);
            chassis.setDefaultCommand(driveChassis);

            GamepadButton buttonA_1 =  pad1.getGamepadButton(GamepadKeys.Button.A);
            buttonA_1.toggleWhenPressed(new TurboDriveCommand(chassis, pad1::getLeftY, pad1::getLeftX, pad1::getRightX));
            ////////
            // also works
            //   |
            //   V
            // buttonA.whileHeld( new TurboMecDriveCommand( chassis, pad1::getLeftY, pad1::getLeftX, pad1::getRightX) );
            ////////

            lift = new LiftSubsystem(hardwareMap, telemetry);
            register(lift);

            DefaultLiftCommand liftUp = new DefaultLiftCommand(true, 0.8, lift);
            DefaultLiftCommand liftDown = new DefaultLiftCommand(false, 0.8, lift);

            GamepadButton buttonY_2 = pad2.getGamepadButton(GamepadKeys.Button.Y);
            GamepadButton buttonA_2 = pad2.getGamepadButton(GamepadKeys.Button.A);
            buttonY_2.whenPressed(liftUp);
            buttonA_2.whenPressed(liftDown);




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

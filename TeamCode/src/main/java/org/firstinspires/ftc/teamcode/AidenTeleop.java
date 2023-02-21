// Makes all code compatible with the FTC variables
package org.firstinspires.ftc.teamcode;

// Specific Libraries of commands to import from the package

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


// Initializes the main framework for the program
@TeleOp(name = "FTCLib_AidenTeleop", group = "Teleop")
public class AidenTeleop extends OpMode

{

    // Calls new Robot and gives it a name
    Thunderbot_2022 robot = new Thunderbot_2022();

    // Makes GamepadEx 1 and 2 from FTC Lib
    GamepadEx exGamepad1 = new GamepadEx(gamepad1);
    GamepadEx exGamepad2 = new GamepadEx(gamepad2);

    // instantiates toggle "B" button and calls it in
    ToggleButtonReader bReader1 = new ToggleButtonReader(
            exGamepad1, GamepadKeys.Button.B
    );

    // Reads the left trigger for gamepad1
    TriggerReader leftTriggerReader1 = new TriggerReader(
            exGamepad1, GamepadKeys.Trigger.LEFT_TRIGGER
    );

    TriggerReader rightTriggerReader1 = new TriggerReader(
            exGamepad1, GamepadKeys.Trigger.RIGHT_TRIGGER
    );

    // Variables for the positions for claw, wrist, and elbow
    double wristPosition = 0;
    double clawPosition = 0;
    double elbowPosition = 0.535;

    double ELBOW_INCREMENT = 0.005;
    double WRIST_INCREMENT = 0.015; // its 0.0025
    double CLAW_INCREMENT = 0.0125;

    // All the things that happen when the init button is pressed
    @Override
    public void init() {
        telemetry.addData("Init", "Start");

        // Calls and initializes all values and parts of the robot declared in ThunderBot_2022
        robot.init(hardwareMap, telemetry, false);

        // Displays Init: Done
        telemetry.addData("Init", "Done");

        // Makes sure the Claw and Wrist can move as free as they need to
        try {
            wristPosition = 0.625;
            clawPosition = 0.5;
            robot.armstrong.elbowMove(elbowPosition);
            robot.armstrong.wristMove(wristPosition);
            robot.armstrong.clawMove(clawPosition);
        } catch (Exception e) {
            telemetry.addData("cant", "run");
        }

    }

    public void init_loop() {
        telemetry.addData("Lelbow Position", robot.armstrong.leftElbow.getPosition());
        telemetry.addData("Relbow Position", robot.armstrong.rightElbow.getPosition());
        telemetry.addData("Wrist Position", robot.armstrong.wrist.getPosition());
        telemetry.addData("Claw Position: ", robot.armstrong.claw.getPosition());
    }

    // All the things it does when you select Play button
    @Override
    public void start() {
        robot.start();
    }

    // All the things it does over and over during the period from when start is pressed to when stop is pressed.
    @Override
    public void loop() {
        robot.update();

        /////////////////
        // CHASSIS
        /////////////////

        // left trigger = 20% power
        // right trigger = 70% power
        // A button = 100% power
        // these are just tenative; we can change them to what joshua prefers.
        // TODO: consult the drivers to see what other speeds they'd like...
        if (exGamepad1.isDown(GamepadKeys.Button.A)) {
            // TURBO!!!
            robot.joystickDrive(-exGamepad1.getLeftY(), exGamepad1.getLeftX(), exGamepad1.getRightX());
        }
//        else if (leftTriggerReader1.isDown()){
//            // 80 percent drive
//            robot.joystickDrive(-exGamepad1.getLeftY() * 0.8, exGamepad1.getLeftX() * 0.8, exGamepad1.getRightX() * 0.8);
//        }
        else if (rightTriggerReader1.isDown()) {
            // 30 percent drive
            robot.joystickDrive(-exGamepad1.getLeftY() * 0.3, exGamepad1.getLeftX() * 0.3, exGamepad1.getRightX() * 0.2);
        } else {
            // Normal Drive
            double sign = Math.signum(exGamepad1.getRightX());
            robot.joystickDrive(-exGamepad1.getLeftY() * 0.6, exGamepad1.getLeftX() * 0.6, exGamepad1.getRightX() * 0.6);
        }



        // toggles "b" button between 80% (speedy) and 60% (normal)
        if (bReader1.getState()) {
            // 80 percent drive
            robot.joystickDrive(-exGamepad1.getLeftY() * 0.8, exGamepad1.getLeftX() * 0.8, exGamepad1.getRightX() * 0.8);
        } else {
            // Normal Drive
            double sign = Math.signum(exGamepad1.getRightX());
            robot.joystickDrive(-exGamepad1.getLeftY() * 0.6, exGamepad1.getLeftX() * 0.6, exGamepad1.getRightX() * 0.6);
        }
        // reads so it can toggle
        bReader1.readValue();



        /////////////////
        // TWIST
        /////////////////

        // TODO: Add manual control/tweaking of twist here.  The automatic twist is now being handled
        // by the Armstrong class.
        
        if (gamepad2.left_stick_button) {
            robot.armstrong.armRotate(1);
        } else if (gamepad2.right_stick_button) {
            robot.armstrong.armRotate(0);
        }
//        if (elbowPosition < 0.31) {
//            robot.armstrong.armRotate(1);
//        }
//        else if (elbowPosition > 0.31) {
//            robot.armstrong.armRotate(0);
//        }
//         less than 0.31 twist reverse
//         more 0.31 untwist

        /////////////////
        // CLAW
        /////////////////
        if (gamepad2.left_bumper) {
            clawPosition += CLAW_INCREMENT;
        } else if (gamepad2.right_bumper) {
            clawPosition -= CLAW_INCREMENT;
        }
        clawPosition = Range.clip(clawPosition, robot.armstrong.getCLAW_MIN(), robot.armstrong.getCLAW_MAX());
        robot.armstrong.clawMove(clawPosition);


        /////////////////
        // ELBOW
        /////////////////
        if (gamepad2.b) {
            elbowPosition -= ELBOW_INCREMENT;
        } else if (gamepad2.x) {
            elbowPosition += ELBOW_INCREMENT;
        }

        elbowPosition = Range.clip(elbowPosition, robot.armstrong.getELB_MIN(), robot.armstrong.getELB_MAX());
        robot.armstrong.elbowMove(elbowPosition);

        /////////////////
        // LINEAR SLIDE
        /////////////////
        if (gamepad2.y) {
            robot.armstrong.liftUp(1);
        } else if (gamepad2.a) {
            robot.armstrong.liftDown(1);
        } else {
            robot.armstrong.liftStop();
        }
        /////////////////
        // WRIST
        /////////////////
        if(gamepad2.dpad_up) {
            wristPosition += WRIST_INCREMENT;
            wristPosition = Range.clip(wristPosition, robot.armstrong.getWRIST_MIN(), robot.armstrong.getWRIST_MAX());
            robot.armstrong.wristMove(wristPosition);
        } else if (gamepad2.dpad_down) {
            wristPosition -= WRIST_INCREMENT;
            wristPosition = Range.clip(wristPosition, robot.armstrong.getWRIST_MIN(), robot.armstrong.getWRIST_MAX());
            robot.armstrong.wristMove(wristPosition);
        }

        if (elbowPosition > 0.495) {
            robot.armstrong.wristMove(0.5);
        }
        /////////////////
        // Sensors
        /////////////////

//        robot.armstrong.detectColor();
//        if (robot.armstrong.color.red() > 200 && robot.armstrong.color.green() > 200
//                && robot.armstrong.color.red() < 400 && robot.armstrong.color.green() < 400) {
//            telemetry.addData("yellow", "is detected");
//        } else {
//            telemetry.addData("yellow", "is not detected");
//        }

       // robot.armstrong.detectDistance();
    }
}

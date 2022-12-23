// Makes all code compatible with the FTC variables
package org.firstinspires.ftc.teamcode;

// Specific Libraries of commands to import from the package
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

// Initializes the main framework for the program
@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode
{

    // Calls new Robot and gives it a name
    Thunderbot_2022 robot = new Thunderbot_2022();

    // Calls two new variables for the positions for claw/wrist
    double wristPosition = 0;
    double clawPosition = 0;
    double lelbowPosition = 0;
    double relbowPosition = 0;
    double ELBOW_INCREMENT = 0.0025;
    double WRIST_INCREMENT = 0.0075; // its 0.0025
    double CLAW_INCREMENT = 0.01;

    // All the things that happen when the init button is pressed
    @Override
    public void init() {
        telemetry.addData("Init", "Start");

        // Calls and initalizes all values and parts of the robot declared in ThunderBot_2022
        robot.init(hardwareMap, telemetry);

        // Displays Init: Done
        telemetry.addData("Init", "Done");

        // Makes sure the Claw and Wrist can move as free as they need to
        try {
            wristPosition = 0.625;
            clawPosition = 0.5;
            lelbowPosition = robot.armstrong.leftElbow.getPosition();
            relbowPosition = robot.armstrong.rightElbow.getPosition();
            robot.armstrong.elbowMove(lelbowPosition, relbowPosition);
            robot.armstrong.wristMove(wristPosition);
            robot.armstrong.clawMove(clawPosition);
        } catch (Exception e) {
            telemetry.addData("cant", "run");
        }
    }

    // All the things it does when you select Play button
    @Override
    public void start() {
        robot.start();
    }

    // All the things it does over and over during the period from when start is pressed to when stop is pressed.
    @Override
    public void loop()
    {
        robot.update();

        /////////////////
        // CHASSIS
        /////////////////
        if (gamepad1.a) {
            // TURBO!!!
            robot.joystickDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            // Normal Drive
            robot.joystickDrive(-gamepad1.left_stick_y * 0.4, gamepad1.left_stick_x * 0.4, gamepad1.right_stick_x * 0.4);
        }

        /////////////////
        // TWIST
        /////////////////
        if (gamepad2.dpad_left) {
            robot.armstrong.armRotate(1);
        } else if (gamepad2.dpad_right) {
            robot.armstrong.armRotate(0);
        }

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
        if (gamepad2.x) {
            lelbowPosition -= ELBOW_INCREMENT;
            relbowPosition -= ELBOW_INCREMENT;
        } else if (gamepad2.b) {
            lelbowPosition += ELBOW_INCREMENT;
            relbowPosition += ELBOW_INCREMENT;
        }

        relbowPosition = Range.clip(relbowPosition, 0, 0.285);
        lelbowPosition = Range.clip(lelbowPosition, 0, 0.285);
        robot.armstrong.elbowMove(lelbowPosition, relbowPosition);

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
    }
}

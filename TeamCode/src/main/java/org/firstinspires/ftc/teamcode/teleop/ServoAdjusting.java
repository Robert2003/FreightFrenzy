package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotDefinition;

@TeleOp(group = "drive")
public class ServoAdjusting extends LinearOpMode {

    Servo servo;
    double moveInterval = 0.0005;
    double servoPos;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = new RobotDefinition(hardwareMap).getExcavator();
        servoPos = 0;
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_left)
                servoPos -= moveInterval;
            else if(gamepad1.dpad_right)
                servoPos += moveInterval;
            servo.setPosition(servoPos);
            telemetry.addData("Servo Position", servoPos);
            telemetry.update();
        }
    }

}

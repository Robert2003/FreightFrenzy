package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotDefinition;

@TeleOp(group = "drive")
public class ServoAdjusting extends LinearOpMode {

    Servo servo;
    double moveInterval = 0.05;

    public ServoAdjusting(){
        servo = new RobotDefinition(hardwareMap).getExcavator();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeIsActive()){
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
            if(gamepad1.dpad_left)
                servo.setPosition(servo.getPosition() - moveInterval);
            else if(gamepad1.dpad_right)
                servo.setPosition(servo.getPosition() + moveInterval);
        }
    }

}

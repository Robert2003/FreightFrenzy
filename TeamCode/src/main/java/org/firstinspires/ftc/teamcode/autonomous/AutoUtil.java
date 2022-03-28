package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.RobotDefinition;

public class AutoUtil {

    public static void setClawOpen(RobotDefinition robot, Servo servo, boolean open) {
        double pos = (open ? robot.getExcavatorOpen() : robot.getExcavatorClosed());
        servo.setPosition(pos);
    }

    public static void armToPosition(DcMotorEx armMotor, int position) {
        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    public static void plateToPosition(DcMotorEx plateMotor, int position) {
        plateMotor.setTargetPosition(position);
        plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        plateMotor.setPower(1);
    }

    public static void rotateDucks(DcMotorEx duckMotor, float power){
        duckMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckMotor.setPower(power);
    }


}

package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "drive")
public class JustArm extends OpMode {

    DcMotorEx armMotor;
    double power = 0.4;
    boolean pressed = false;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setZeroPowerBehavior(BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad2.left_trigger != 0) {
            armMotor.setPower(power);
        } else if (gamepad2.right_trigger != 0) {
            armMotor.setPower(-power);
        } else {
            armMotor.setPower(0);
        }


        if(gamepad1.y || gamepad1.a)
        {
            if(gamepad1.y && !pressed)
            {
                pressed = true;
                power+=0.1;
            }
            else if(gamepad1.a && !pressed)
            {
                pressed = true;
                power-=0.1;
            }
        }
        else
            pressed = false;

        telemetry.addData("Power", power);
        telemetry.update();
    }
}

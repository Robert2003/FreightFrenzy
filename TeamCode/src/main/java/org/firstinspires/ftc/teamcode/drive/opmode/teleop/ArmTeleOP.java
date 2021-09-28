package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class ArmTeleOP extends OpMode {

    DcMotorEx plateMotor, armMotor;

    @Override
    public void init() {
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            plateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            plateMotor.setVelocity(25);
        } else if (gamepad1.b) {
            plateMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            plateMotor.setVelocity(25);
        } else
            plateMotor.setVelocity(0);
        if (gamepad1.right_bumper) {
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setVelocity(25);
        } else if (gamepad1.left_bumper) {
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setVelocity(25);
        } else
            armMotor.setVelocity(0);
    }

    /*
    @Override
    public void runOpMode() throws InterruptedException {

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a)
                plateMotor.setPower(0.1);
            if (gamepad1.right_bumper) {
                //armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                armMotor.setPower(0.1);
            } else if (gamepad1.left_bumper) {
                //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                armMotor.setPower(-0.1);
            }
        }

    }
    */

}

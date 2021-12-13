package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "drive")
@Disabled
public class ArmTeleOPBackup extends OpMode {

    DcMotorEx plateMotor, armMotor;
    DcMotorEx rearLeftMotor, frontLeftMotor, rearRightMotor, frontRightMotor;
    double drive, strafe, rotate;
    double rearLeftPower, frontLeftPower, rearRightPower, frontRightPower;

    @Override
    public void init() {
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, "rearLeftMotor");
        rearRightMotor = hardwareMap.get(DcMotorEx.class, "rearRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        drive = gamepad1.right_stick_y;
        strafe = -gamepad1.right_stick_x;
        rotate = -gamepad1.right_trigger + gamepad1.left_trigger;

        rearLeftPower = -strafe + drive + rotate;
        frontLeftPower = strafe + drive + rotate;
        rearRightPower = strafe + drive - rotate;
        frontRightPower = -strafe + drive - rotate;

        rearLeftPower = Range.clip(rearLeftPower, -1.0, 1.0);
        rearRightPower = Range.clip(rearRightPower, -1.0, 1.0);
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);

        if(gamepad1.right_bumper){
            rearLeftPower *= 0.3;
            rearRightPower *= 0.3;
            frontLeftPower *= 0.3;
            frontRightPower *= 0.3;
        }

        rearLeftMotor.setPower(rearLeftPower);
        rearRightMotor.setPower(rearRightPower);
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);

        if (gamepad2.a) {
            plateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            plateMotor.setVelocity(0.1);
        } else if (gamepad2.b) {
            plateMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            plateMotor.setVelocity(0.1);
        } else {
            plateMotor.setTargetPosition(plateMotor.getCurrentPosition());
        }
        if (gamepad2.right_bumper) {
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setVelocity(0.1);
        } else if (gamepad2.left_bumper) {
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setVelocity(0.1);
        } else {
            plateMotor.setTargetPosition(plateMotor.getCurrentPosition());
        }
    }

}
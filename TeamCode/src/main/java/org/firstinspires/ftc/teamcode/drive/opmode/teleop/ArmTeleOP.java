package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class ArmTeleOP extends OpMode {

    DcMotorEx plateMotor, armMotor;
    DcMotorEx rearLeftMotor, frontLeftMotor, rearRightMotor, frontRightMotor;
    Servo excavator;

    int platePosition, armPosition;

    double drive, strafe, rotate;
    double rearLeftPower, frontLeftPower, rearRightPower, frontRightPower;
    double servoPos = 0;

    @Override
    public void init() {
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, "rearLeftMotor");
        rearRightMotor = hardwareMap.get(DcMotorEx.class, "rearRightMotor");

        excavator = hardwareMap.get(Servo.class, "servo");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        plateMotor.setZeroPowerBehavior(BRAKE);
        armMotor.setZeroPowerBehavior(BRAKE);

        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        drive = -gamepad1.right_stick_x;
        strafe = -gamepad1.right_stick_y;
        rotate = -gamepad1.right_trigger + gamepad1.left_trigger;

        rearLeftPower = -strafe - drive + rotate;
        frontLeftPower = strafe + drive + rotate;
        rearRightPower = strafe - drive - rotate;
        frontRightPower = -strafe + drive - rotate;

        rearLeftPower = Range.clip(rearLeftPower, -1.0, 1.0);
        rearRightPower = Range.clip(rearRightPower, -1.0, 1.0);
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);

        if(gamepad1.left_bumper){
            rearLeftPower *= 0.7;
            rearRightPower *= 0.7;
            frontLeftPower *= 0.7;
            frontRightPower *= 0.7;
        }
        if (gamepad1.right_bumper) {
            rearLeftPower *= 0.3;
            rearRightPower *= 0.3;
            frontLeftPower *= 0.3;
            frontRightPower *= 0.3;
        }

        rearLeftMotor.setPower(rearLeftPower);
        rearRightMotor.setPower(rearRightPower);
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);


        if (gamepad2.right_stick_x != 0) {
            plateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            plateMotor.setVelocity(120 * gamepad2.right_stick_x);
            platePosition = plateMotor.getCurrentPosition();
        } else {
            plateMotor.setTargetPosition(platePosition);
            plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            plateMotor.setVelocity(200);
        }

        if (gamepad2.left_trigger != 0) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setVelocity(170);
            armPosition = armMotor.getCurrentPosition();
        } else if (gamepad2.right_trigger != 0) {
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setVelocity(170);
            armPosition = armMotor.getCurrentPosition();
        } else {
            armMotor.setTargetPosition(armPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setVelocity(100);
        }

        if (gamepad2.dpad_left) {
            servoPos = 1.0;
        } else if (gamepad2.dpad_right)
            servoPos = 0;
        excavator.setPosition(servoPos);

        telemetry.addData("Excavator target", servoPos);
        telemetry.addData("Excavator actual", excavator.getPosition());
        telemetry.addData("Arm position", armMotor.getCurrentPosition());
        telemetry.addData("Arm RunMode", armMotor.getMode());
        telemetry.addData("Plate position", plateMotor.getCurrentPosition());
        telemetry.addData("Plate power", plateMotor.getPower());
        telemetry.addData("Plate RunMode", plateMotor.getMode());
        telemetry.update();

    }
}

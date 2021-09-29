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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class ArmTeleOP extends OpMode {

    DcMotorEx plateMotor, armMotor;
    int platePosition, armPosition;

    @Override
    public void init() {
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        plateMotor.setZeroPowerBehavior(BRAKE);
        armMotor.setZeroPowerBehavior(BRAKE);

        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        /**if (gamepad1.a) {
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
         */
        if(gamepad1.a || gamepad1.b || gamepad1.right_bumper || gamepad1.left_bumper)
        {
            plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (gamepad1.a)
                plateMotor.setPower(0.1);
            else if (gamepad1.b)
                plateMotor.setPower(-0.1);

            if (gamepad1.right_bumper)
                armMotor.setPower(0.1);
            else if (gamepad1.left_bumper)
                armMotor.setPower(-0.1);

            platePosition = plateMotor.getCurrentPosition();
            armPosition = armMotor.getCurrentPosition();
        }
        else
        {
            //plateMotor.setPower(0);
            //armMotor.setPower(0);

            telemetry.addData("Pozitie din variabila:", platePosition);
            telemetry.addData("Pozitie din motor:", plateMotor.getCurrentPosition());
            telemetry.addData("Pozitie din variabila:", armPosition);
            telemetry.addData("Pozitie din motor:", armMotor.getCurrentPosition());
            telemetry.update();

            plateMotor.setTargetPosition(platePosition);
            plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            plateMotor.setPower(0.1);

            armMotor.setTargetPosition(armPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.1);
        }
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

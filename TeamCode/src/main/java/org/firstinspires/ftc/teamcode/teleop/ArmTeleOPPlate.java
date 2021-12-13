package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "drive")
public class ArmTeleOPPlate extends OpMode {

    DcMotorEx armMotor, plateMotor;
    DcMotorEx rearLeftMotor, frontLeftMotor, rearRightMotor, frontRightMotor;
    Servo excavator;

    int platePosition, armPosition;

    double drive, strafe, rotate;
    double rearLeftPower, frontLeftPower, rearRightPower, frontRightPower;
    double servoPos = 0;
    boolean pressed = false;

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

        if (gamepad1.left_bumper) {
            rearLeftPower *= 0.5;
            rearRightPower *= 0.5;
            frontLeftPower *= 0.5;
            frontRightPower *= 0.5;
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

        //PLATE - WITHOUT ENCODER
        if(!plateMotor.isBusy())
        {
            if (gamepad2.right_stick_x > 0) {
                plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                plateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                plateMotor.setPower(1);
            } else if (gamepad2.right_stick_x < 0) {
                plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                plateMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                plateMotor.setPower(1);
            } else {
                plateMotor.setPower(0);
            }
        }

        //PLATE - RUN WITH ENCODER
        if(gamepad2.left_bumper){
            plateMotor.setTargetPosition(-3000);
            plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            plateMotor.setPower(1);
        } else if(gamepad2.right_bumper){
            plateMotor.setTargetPosition(0);
            plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            plateMotor.setPower(1);
        }

        //ARM MANUAL - WITHOUT ENCODER
        if(!armMotor.isBusy())
        {
            if (gamepad2.left_trigger != 0) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                armMotor.setPower(1);
            } else if (gamepad2.right_trigger != 0) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                armMotor.setPower(1);
            } else {
                armMotor.setPower(0);
            }
        }

        //ARM AUTOMATIC
        if (gamepad2.dpad_up) {
            armMotor.setTargetPosition(1350);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
        } else if (gamepad2.dpad_down) {
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
        }

        //CLAW
        /**if (gamepad2.dpad_left)
         {
         if(!pressed)
         {
         servoPos += 0.05;
         pressed = true;
         }
         } else if (gamepad2.dpad_right)
         {
         if(!pressed)
         {
         servoPos -=0.05;
         pressed = true;
         }
         }
         else
         pressed = false;
         if(servoPos<0)
         servoPos = 0;
         if(servoPos>1)
         servoPos = 1;*/
        if(gamepad2.dpad_left)
            servoPos = 0;
        else if(gamepad2.dpad_right)
            servoPos = 0.15;
        excavator.setPosition(servoPos);

        telemetry.addData("Excavator target", servoPos);
        telemetry.addData("Excavator actual", excavator.getPosition());
        telemetry.addData("Arm position", armMotor.getCurrentPosition());
        telemetry.addData("Arm RunMode", armMotor.getMode());
        telemetry.addData("Plate position", plateMotor.getCurrentPosition());
        telemetry.addData("Plate power", plateMotor.getPower());
        telemetry.addData("Plate RunMode", plateMotor.getMode());
        telemetry.addData("Is Busy", armMotor.isBusy());
        telemetry.update();
    }

}
package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.Stack;

@TeleOp(group = "drive")
public class FreightFrenzyTeleOp extends LinearOpMode {

    DcMotorEx armMotor, plateMotor;
    DcMotorEx rearLeftMotor, frontLeftMotor, rearRightMotor, frontRightMotor;
    Servo excavator;

    double drive, strafe, rotate;
    double rearLeftPower, frontLeftPower, rearRightPower, frontRightPower;
    double servoPos = 0;

    //SampleMecanumDrive mecanumDrive;

    private class MoveTarget {
        private DcMotorEx motor;
        private int position;

        public MoveTarget(DcMotorEx motor, int position) {
            this.motor = motor;
            this.position = position;
        }

        public DcMotorEx getMotor() {
            return motor;
        }

        public int getPosition() {
            return position;
        }
    }

    Queue<MoveTarget> moveTargets = new LinkedList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
        initialization();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("leftsticky", -gamepad1.left_stick_y);
            telemetry.addData("leftstickx", -gamepad1.left_stick_x);
            telemetry.addData("rightstickx(heading)", -gamepad1.right_stick_x);
            telemetry.update();

            mecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            mecanumDrive.update();
        }
    }

    private void initialization() {
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        rearRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        excavator = hardwareMap.get(Servo.class, "servo");

        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        plateMotor.setZeroPowerBehavior(BRAKE);
        armMotor.setZeroPowerBehavior(BRAKE);

        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        rearLeftMotor.setZeroPowerBehavior(BRAKE);
        rearRightMotor.setZeroPowerBehavior(BRAKE);

        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //mecanumDrive = new SampleMecanumDrive(hardwareMap);
        //mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
    }

    private void run() {
        //controlDriving();
        //controlClaw();
        //controlArm();
        //debugTelemetry();
    }

    private void controlClaw() {
        if (gamepad1.dpad_left)
            servoPos = 0.15;
        else if (gamepad1.dpad_right)
            servoPos = 0;
        excavator.setPosition(servoPos);
    }

    private void debugTelemetry() {
        //telemetry.addData("positionX", mecanumDrive.getPoseEstimate().getX());
        //telemetry.addData("positionY", mecanumDrive.getPoseEstimate().getY());
        //telemetry.update();
    }

    private void controlDriving() {
        /**drive = gamepad1.right_stick_x;
        strafe = -gamepad1.right_stick_y;
        rotate = -gamepad1.right_trigger + gamepad1.left_trigger;

        telemetry.addData("drive", drive);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.update();

        frontLeftPower = strafe + drive + rotate;
        rearLeftPower = strafe + drive - rotate;
        rearRightPower = strafe - drive + rotate;
        frontRightPower = strafe - drive - rotate;

        /*double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = y - x + rx;
        double powerBackRight = y + x - rx;



        rearLeftPower = Range.clip(rearLeftPower, -1.0, 1.0);
        rearRightPower = Range.clip(rearRightPower, -1.0, 1.0);
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);

        if (gamepad1.right_bumper) {
            rearLeftPower *= 0.5;
            rearRightPower *= 0.5;
            frontLeftPower *= 0.5;
            frontRightPower *= 0.5;
        }
        if (gamepad1.left_bumper) {
            rearLeftPower *= 0.3;
            rearRightPower *= 0.3;
            frontLeftPower *= 0.3;
            frontRightPower *= 0.3;
        }

        frontLeftMotor.setPower(frontLeftPower);
        rearLeftMotor.setPower(rearLeftPower);
        rearRightMotor.setPower(rearRightPower);
        frontRightMotor.setPower(frontRightPower);
        */

        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("leftsticky", -gamepad1.left_stick_y);
        telemetry.addData("leftstickx", -gamepad1.left_stick_x);
        telemetry.addData("rightstickx(heading)", -gamepad1.right_stick_x);
        telemetry.update();

        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        mecanumDrive.update();
    }

    private void executeCurrentMoveTarget() {
        if(moveTargets.isEmpty()) return;

        MoveTarget moveTarget = moveTargets.peek();
        DcMotorEx motor = moveTarget.getMotor();

        motor.setTargetPosition(moveTarget.getPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

        if (!motor.isBusy())
            moveTargets.remove();
    }

    boolean armSuppress;
    double armPower, platePower;

    private void controlArm() {
        executeCurrentMoveTarget();
        MoveTarget currentTarget;
        if (gamepad2.a) {
            moveTargets.clear();
            currentTarget = new MoveTarget(plateMotor, 0);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(armMotor, 0);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.x) {
            moveTargets.clear();
            currentTarget = new MoveTarget(armMotor, 430); //600
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -2900);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.b){
            moveTargets.clear();
            currentTarget = new MoveTarget(armMotor, 800); // 1000
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -2900);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.y){
            moveTargets.clear();
            currentTarget = new MoveTarget(armMotor, 1160); //1660
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -2900);
            moveTargets.add(currentTarget);
        }

        if (gamepad2.dpad_down) {
            moveTargets.clear();
            currentTarget = new MoveTarget(plateMotor, 0);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(armMotor, 0);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.dpad_left) {
            moveTargets.clear();
            currentTarget = new MoveTarget(armMotor, 600);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 2900);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.dpad_right){
            moveTargets.clear();
            currentTarget = new MoveTarget(armMotor, 1000);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 2900);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.dpad_up){
            moveTargets.clear();
            currentTarget = new MoveTarget(armMotor, 1660);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 2900);
            moveTargets.add(currentTarget);
        }



        armSuppress = gamepad2.right_bumper;

        if(gamepad2.right_stick_y != 0){
            moveTargets.clear();
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armPower = (armSuppress ? .25 : .8);
            if(gamepad2.right_stick_y > 0)
                armPower *= -1;
            armMotor.setPower(armPower);
        } else if(armMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            armMotor.setPower(0);

        if (gamepad2.right_stick_x != 0) {
            moveTargets.clear();
            plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            platePower = (armSuppress ? .25 : .8);
            if(gamepad2.right_stick_x > 0)
                platePower *= -1;
            plateMotor.setPower(platePower);
        } else if(plateMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            plateMotor.setPower(0);

    }
}

package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    boolean isUpRight = false, isUpLeft = false, releasedDpad = true;

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
        initialization();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
    }

    private void initialization() {
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

        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        rearLeftMotor.setZeroPowerBehavior(BRAKE);
        rearRightMotor.setZeroPowerBehavior(BRAKE);

        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //createArmPositions();
    }

    /**private void createArmPositions() {
        armPositions.put(ArmLevel.ZERO, 0);
        armPositions.put(ArmLevel.LOW, 300);
        armPositions.put(ArmLevel.MID, 1000);
        armPositions.put(ArmLevel.HIGH, 1500);
        armPositions.put(ArmLevel.CAP, 1700);
    }*/

    private void run() {
        controlDriving();
        controlClaw();
        controlArm();
    }

    private void controlClaw() {
        if (gamepad1.dpad_left)
            servoPos = 0.15;
        else if (gamepad1.dpad_right)
            servoPos = 0;
        excavator.setPosition(servoPos);
    }

    private void debugTelemetry() {

        telemetry.update();
    }

    private void controlDriving() {
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

        rearLeftMotor.setPower(rearLeftPower);
        rearRightMotor.setPower(rearRightPower);
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
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
            currentTarget = new MoveTarget(armMotor, 600);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -2900);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.b){
            moveTargets.clear();
            currentTarget = new MoveTarget(armMotor, 1000);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -2900);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.y){
            moveTargets.clear();
            currentTarget = new MoveTarget(armMotor, 1660);
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
    }
}

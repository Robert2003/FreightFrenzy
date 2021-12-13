package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(group = "drive")
public class FreightFrenzyTeleOpBackupRoadrunner extends LinearOpMode {

    DcMotorEx armMotor, plateMotor;
    Servo excavator;

    SampleMecanumDrive mecanumDrive;
    double suppressedPower;

    double servoPos = 0;

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
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        initialization();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
    }

    private void initialization() {
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        excavator = hardwareMap.get(Servo.class, "servo");

        plateMotor.setZeroPowerBehavior(BRAKE);
        armMotor.setZeroPowerBehavior(BRAKE);

        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    private void run() {
        controlDriving();
        controlClaw();
        controlArm();
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
        telemetry.addData("Queue size", moveTargets.size());
        if(moveTargets.size() > 0)
            telemetry.addData("Queue front motor", moveTargets.peek().getMotor().getPower());
        telemetry.update();
    }

    private void controlDriving() {
        suppressedPower = 1;
        if(gamepad1.right_bumper)
            suppressedPower *= .5;
        if(gamepad1.left_bumper)
            suppressedPower *= .3;
        telemetry.addData("suppress", suppressedPower);
        telemetry.addData("sticky", gamepad1.right_stick_y);
        telemetry.addData("stickx", gamepad1.right_stick_x);
        telemetry.update();
        mecanumDrive.setDrivePower(
                new Pose2d(
                        -gamepad1.right_stick_y * suppressedPower,
                        -gamepad1.right_stick_x * suppressedPower,
                        (-gamepad1.right_trigger + gamepad1.left_trigger) * suppressedPower
                )
        );
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

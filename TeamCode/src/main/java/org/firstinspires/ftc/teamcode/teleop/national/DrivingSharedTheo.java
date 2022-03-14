package org.firstinspires.ftc.teamcode.teleop.national;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(group = "Driving")
public class DrivingSharedTheo extends LinearOpMode {

    RobotDefinition robot;

    DcMotorEx armMotor, plateMotor, flyWheel;
    //DcMotorEx rearLeftMotor, frontLeftMotor, rearRightMotor, frontRightMotor;
    Servo excavator;

    double servoPos = 0;

    SampleMecanumDrive mecanumDrive;

    ElapsedTime runtime = new ElapsedTime();

    boolean sharedControls = false;

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
        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
    }

    private void initialization() {
        robot = new RobotDefinition(hardwareMap);

        plateMotor = robot.getPlateMotor();
        armMotor = robot.getArmMotor();
        flyWheel = robot.getFlyWheel();

        excavator = robot.getExcavator();

        plateMotor.setZeroPowerBehavior(BRAKE);
        armMotor.setZeroPowerBehavior(BRAKE);

        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.setZeroPowerBehavior(BRAKE);
    }

    private void run() {
        controlDriving();
        controlClaw();
        controlArm();
        controlContinuousServo();
        debugTelemetry();
    }

    private void controlClaw() {
        if (gamepad1.dpad_left)
            servoPos = 0.15;
        else if (gamepad1.dpad_right)
            servoPos = 0;
        excavator.setPosition(servoPos);
    }

    private void debugTelemetry() {
        telemetry.addData("Controls", sharedControls ? "SHARED" : "VANILLA");
        telemetry.addData("PoseX", mecanumDrive.getPoseEstimate().getX());
        telemetry.addData("PoseY", mecanumDrive.getPoseEstimate().getY());
        telemetry.addData("Arm", armMotor.getCurrentPosition());
        telemetry.addData("Plate", plateMotor.getCurrentPosition());
        telemetry.addData("Encoder0", mecanumDrive.getWheelPositions().get(0));
        telemetry.addData("Encoder1", mecanumDrive.getWheelPositions().get(1));
        telemetry.addData("Encoder2", mecanumDrive.getWheelPositions().get(2));
        telemetry.addData("Encoder3", mecanumDrive.getWheelPositions().get(3));
        telemetry.addData("ArmMode", armMotor.getMode());
        telemetry.addData("PlateMode", plateMotor.getMode());
        telemetry.addData("ArmPower", armMotor.getPower());
        telemetry.addData("PlatePower", plateMotor.getPower());
        telemetry.update();
    }

    double swp, ms; //suppressed wheel power

    long lastSwitchTimeSec = 0;

    private void controlDriving() {

        ms = .82f;

        if(gamepad1.dpad_up)
            ms = 1;
        swp = 1;
        if(gamepad1.right_bumper)
            swp *= .4;
        if(gamepad1.left_bumper)
            swp *= .3;

        if(!sharedControls) {
            mecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.right_stick_y * swp * ms,
                            -gamepad1.right_stick_x * swp * ms,
                            (-gamepad1.right_trigger + gamepad1.left_trigger) * swp * ms
                    )
            );
        } else{
            mecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.right_stick_y * swp * ms,
                            gamepad1.right_stick_x * swp * ms,
                            (-gamepad1.right_trigger + gamepad1.left_trigger) * swp * ms
                    )
            );
        }

        if(gamepad1.y){
            sharedControls = true;
        } else if(gamepad1.x)
            sharedControls = false;

        mecanumDrive.update();
    }

    private void executeCurrentMoveTarget() {
        if(moveTargets.isEmpty()) return;

        MoveTarget moveTarget = moveTargets.peek();
        DcMotorEx motor = moveTarget.getMotor();

        motor.setTargetPosition(moveTarget.getPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

        if(!sharedControls) {
            if (runtime.milliseconds() > 700) {//800
                moveTargets.remove();
                runtime.reset();
            }
        }
        else {
            if (runtime.milliseconds() > 400) {
                moveTargets.remove();
                runtime.reset();
            }
        }
    }

    private void resetTargets(){
        moveTargets.clear();
        runtime.reset();
    }

    boolean armSuppress;
    double armPower, platePower;
    int armPosition, platePosition;

    private void controlArm() {
        executeCurrentMoveTarget();
        MoveTarget currentTarget;
        if(!sharedControls) {
            if (gamepad2.dpad_down) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 550);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, 0);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(armMotor, 15);
                moveTargets.add(currentTarget);
            } else if (gamepad2.dpad_left) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 550);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, -1305);
                moveTargets.add(currentTarget);
            } else if (gamepad2.dpad_right) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 1225);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, -1305);
                moveTargets.add(currentTarget);
            } else if (gamepad2.dpad_up) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 1825);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, -1305);
                moveTargets.add(currentTarget);
            } else if (gamepad2.a) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 550);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, 0);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(armMotor, 15);
                moveTargets.add(currentTarget);
            } else if(gamepad2.x) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 550);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, 1305);
                moveTargets.add(currentTarget);
            } else if(gamepad2.b){
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 1225);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, 1305);
                moveTargets.add(currentTarget);
            } else if(gamepad2.y){
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 1825);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, 1305);
                moveTargets.add(currentTarget);
            }
        } else{
            if (gamepad2.dpad_down) {
                resetTargets();
                currentTarget = new MoveTarget(plateMotor, 0);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(armMotor, 15);
                moveTargets.add(currentTarget);
            } else if (gamepad2.dpad_left) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 570);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, 1305);
                moveTargets.add(currentTarget);
            } else if (gamepad2.dpad_right) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 1225);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, 1305);
                moveTargets.add(currentTarget);
            } else if (gamepad2.dpad_up) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 770);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, 1305);
                moveTargets.add(currentTarget);
            } else if (gamepad2.a) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 550);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, 0);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(armMotor, 15);
                moveTargets.add(currentTarget);
            } else if(gamepad2.x) {
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 550);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, -1305);
                moveTargets.add(currentTarget);
            } else if(gamepad2.b){
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 1225);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, -1305);
                moveTargets.add(currentTarget);
            } else if(gamepad2.y){
                resetTargets();
                currentTarget = new MoveTarget(armMotor, 1825);
                moveTargets.add(currentTarget);
                currentTarget = new MoveTarget(plateMotor, -1305);
                moveTargets.add(currentTarget);
            }
        }

        if(gamepad2.right_stick_button){
            resetTargets();
            currentTarget = new MoveTarget(armMotor, 2450);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 0);
            moveTargets.add(currentTarget);
        }
        if(gamepad2.left_bumper){
            resetTargets();
            currentTarget = new MoveTarget(armMotor, 475);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 0);
            moveTargets.add(currentTarget);
        }

        armSuppress = gamepad2.right_bumper;

        if(gamepad2.right_stick_y != 0){
            resetTargets();
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armPower = (armSuppress ? .25 : .8);
            if(gamepad2.right_stick_y > 0)
                armPower *= -1;
            armMotor.setPower(armPower);
            armPosition = armMotor.getCurrentPosition();
        } else if(armMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            armMotor.setPower(0);

        if (gamepad2.right_stick_x != 0) {
            resetTargets();
            plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            platePower = (armSuppress ? .25 : .8);
            if(gamepad2.right_stick_x < 0)
                platePower *= -1;
            plateMotor.setPower(platePower);
            platePosition = plateMotor.getCurrentPosition();
        } else if(plateMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            plateMotor.setPower(0);

        if(gamepad2.left_stick_button) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    boolean startedDucks = false;
    float initDuckSpeed = .8f;
    int timeToAccel = 1250, timeToAccelInsta = 700; //millis, era 800 650

    private void controlContinuousServo(){
        if(gamepad2.right_trigger != 0) {
            if(!startedDucks){
                runtime.reset();
                startedDucks = true;
            }
            flyWheel.setPower(initDuckSpeed);
            if(runtime.milliseconds() > timeToAccelInsta)
                flyWheel.setPower(1);
            else if(runtime.milliseconds() > timeToAccel)
                flyWheel.setPower(runtime.milliseconds() * initDuckSpeed / timeToAccel);
        } else if(gamepad2.left_trigger != 0) {
            if(!startedDucks){
                runtime.reset();
                startedDucks = true;
            }
            flyWheel.setPower(-initDuckSpeed);
            if(runtime.milliseconds() > timeToAccelInsta)
                flyWheel.setPower(-1);
            if(runtime.milliseconds() > timeToAccel)
                flyWheel.setPower(-runtime.milliseconds() * initDuckSpeed / timeToAccel);
        } else {
            flyWheel.setPower(0);
            startedDucks = false;
        }
    }

}

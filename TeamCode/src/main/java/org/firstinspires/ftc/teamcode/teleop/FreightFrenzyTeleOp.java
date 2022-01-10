package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(group = "drive")
public class FreightFrenzyTeleOp extends LinearOpMode {

    RobotDefinition robot;

    DcMotorEx armMotor, plateMotor;
    //DcMotorEx rearLeftMotor, frontLeftMotor, rearRightMotor, frontRightMotor;
    Servo excavator;
    CRServo crServo;

    double servoPos = 0;

    SampleMecanumDrive mecanumDrive;

    ElapsedTime runtime = new ElapsedTime();

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

        /*
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        rearRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
         */

        excavator = robot.getExcavator();
        crServo = robot.getCrServo();

        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

    double swp; //suppressed wheel power

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

        swp = 1;
        if(gamepad1.right_bumper)
            swp *= .4;
        if(gamepad1.left_bumper)
            swp *= .3;
        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.right_stick_y * swp,
                        -gamepad1.right_stick_x * swp,
                        (-gamepad1.right_trigger + gamepad1.left_trigger) * swp
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


        if (runtime.milliseconds() > 800) {
            moveTargets.remove();
            runtime.reset();
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
        if (gamepad2.a) {
            resetTargets();
            currentTarget = new MoveTarget(plateMotor, 0);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(armMotor, 15);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.x) {
            resetTargets();
            currentTarget = new MoveTarget(armMotor, 380); //600
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 960);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.b){
            resetTargets();
            currentTarget = new MoveTarget(armMotor, 775); // 1000
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 960);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.y){
            resetTargets();
            currentTarget = new MoveTarget(armMotor, 1250); //1660 -1300
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 960);
            moveTargets.add(currentTarget);
        }

        if (gamepad2.dpad_down) {
            resetTargets();
            currentTarget = new MoveTarget(plateMotor, 0);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(armMotor, 15);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.dpad_left) {
            resetTargets();
            currentTarget = new MoveTarget(armMotor, 380);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -960);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.dpad_right){
            resetTargets();
            currentTarget = new MoveTarget(armMotor, 775);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -960);
            moveTargets.add(currentTarget);
        }
        else if(gamepad2.dpad_up){
            resetTargets();
            currentTarget = new MoveTarget(armMotor, 1250);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -960);
            moveTargets.add(currentTarget);
        }

        if(gamepad2.right_stick_button){
            resetTargets();
            currentTarget = new MoveTarget(armMotor, 1250);
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

        //limiter
        /*if(!gamepad2.left_stick_button) {
            if (armMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                if (armPower > 0 && armPosition <= 10)
                    armMotor.setPower(0);
                telemetry.addData("T", "a incercat");
                telemetry.update();
            }
            if (plateMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                if (platePower > 0 && platePosition >= 1000)
                    plateMotor.setPower(0);
                else if (platePower < 0 && platePosition <= -1000)
                    plateMotor.setPower(0);
                telemetry.addData("T", "a incercat");
                telemetry.update();
            }
        }*/

    }

    private void controlContinuousServo(){
        if(gamepad2.right_trigger != 0)
            crServo.setPower(1);
        else if(gamepad2.left_trigger != 0)
            crServo.setPower(-1);
        else

            crServo.setPower(0);
    }

}

package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;

@TeleOp(group = "drive")
@Disabled
public class ArmTeleOPPlateBackup extends LinearOpMode {

    DcMotorEx armMotor, plateMotor;
    DcMotorEx rearLeftMotor, frontLeftMotor, rearRightMotor, frontRightMotor;
    Servo excavator;

    double drive, strafe, rotate;
    double rearLeftPower, frontLeftPower, rearRightPower, frontRightPower;
    double servoPos = 0;
    boolean isUpRight = false, isUpLeft = false, releasedDpad = true;

    int armLevel = 3;
    Map<ArmLevel, Integer> armPositions = new HashMap<>();

    enum ArmLevel{
        HIGH,
        MID,
        LOW,
        ZERO
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
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

        createArmPositions();
    }

    private void createArmPositions(){
        armPositions.put(ArmLevel.HIGH, 1450);
        armPositions.put(ArmLevel.MID, 1000);
        armPositions.put(ArmLevel.LOW, 300);
        armPositions.put(ArmLevel.ZERO, 0);
    }

    private void goToArmLevel(int armLevel){
        armMotor.setTargetPosition(armPositions.get(ArmLevel.values()[armLevel]));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }

    private void run() {

        controlDriving();
        controlArmPlate();
        controlArmLevel();
        controlClaw();
        debugTelemetry();

    }

    private void controlClaw(){
        if(gamepad2.dpad_left)
            servoPos = 0;
        else if(gamepad2.dpad_right)
            servoPos = 0.15;
        excavator.setPosition(servoPos);
    }

    private void controlArmPlate(){
        if (gamepad2.right_bumper) {
            if(isUpRight){
                plateMotor.setTargetPosition(0);
                plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                plateMotor.setPower(1);
                if(!plateMotor.isBusy()){
                    armLevel = 3;
                    goToArmLevel(armLevel);
                    isUpRight = false;
                    isUpLeft = false;
                }
            } else {
                armMotor.setTargetPosition(armLevel);
                goToArmLevel(armLevel);
                if(!armMotor.isBusy()) {
                    plateMotor.setTargetPosition(-2950);
                    plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    plateMotor.setPower(1);
                    isUpRight = true;
                }
            }
            //while(gamepad2.right_bumper&&!isStopRequested());
        } else if(gamepad2.left_bumper) {
            if(isUpLeft){
                plateMotor.setTargetPosition(0);
                plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                plateMotor.setPower(1);
                if(!plateMotor.isBusy()) {
                    armLevel = 3;
                    goToArmLevel(armLevel);
                    isUpLeft = false;
                    isUpRight = false;
                }
            } else {
                armMotor.setTargetPosition(armLevel);
                goToArmLevel(armLevel);
                armMotor.setPower(1);
                if(!armMotor.isBusy()) {
                    plateMotor.setTargetPosition(2950);
                    plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    plateMotor.setPower(1);
                    isUpLeft = true;
                }
            }
            //while(gamepad2.left_bumper&&!isStopRequested());
        }
    }

    private void debugTelemetry(){
        telemetry.addData("Arm Level", armLevel);
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

    private void controlDriving(){
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

    private void controlArmLevel(){
        if(releasedDpad) {
            if (gamepad2.dpad_up && armLevel > 0)
                armLevel--;
            else if (gamepad2.dpad_down && armLevel < 3)
                armLevel++;
            goToArmLevel(armLevel);
        }
        if(gamepad2.dpad_up || gamepad2.dpad_down)
            releasedDpad = false;
        else
            releasedDpad = true;
        telemetry.addData("dd", ArmLevel.values()[armLevel]);
        telemetry.addData("len", ArmLevel.values().length);
    }

}

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotDefinition {

    DcMotorEx armMotor, plateMotor;
    Servo excavator;

    public RobotDefinition(HardwareMap hardwareMap){
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        excavator = hardwareMap.get(Servo.class, "servo");

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public DcMotorEx getArmMotor() {
        return armMotor;
    }

    public DcMotorEx getPlateMotor() {
        return plateMotor;
    }

    public Servo getExcavator() {
        return excavator;
    }

}

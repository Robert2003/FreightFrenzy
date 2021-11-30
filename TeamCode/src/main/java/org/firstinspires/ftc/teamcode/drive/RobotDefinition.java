package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotDefinition {

    DcMotorEx armMotor, plateMotor;
    Servo excavator;

    public RobotDefinition(HardwareMap hardwareMap){
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        excavator = hardwareMap.get(Servo.class, "servo");
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

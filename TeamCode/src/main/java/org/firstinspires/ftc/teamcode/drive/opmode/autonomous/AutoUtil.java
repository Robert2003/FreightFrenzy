package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.qualcomm.robotcore.hardware.Servo;

public class AutoUtil {

    public static void setClawOpen(Servo servo, boolean open){
        double pos = (open ? 0 : .15);
        servo.setPosition(pos);
    }

}

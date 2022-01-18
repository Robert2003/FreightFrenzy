package org.firstinspires.ftc.teamcode.autonomous.cases;

import android.view.autofill.AutofillId;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.detection.FrenzyDetection;

public class Case1 {

    FrenzyDetection auto;

    Trajectory shippingTraj1;

    public Case1(FrenzyDetection auto){
        this.auto = auto;
        shippingTraj1 = auto.getMecanumDrive()
                .trajectoryBuilder(auto.getMecanumDrive().getPoseEstimate())
                .lineTo(new Vector2d(18.655, 31.861))
                .build();
    }

    public void runAuto(){
        AutoUtil.setClawOpen(auto.getRobot().getExcavator(), false);
        AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 600);
        //while(auto.getRobot().getArmMotor().isBusy());
        auto.sleep(3000);
        auto.getMecanumDrive().followTrajectory(shippingTraj1);
        auto.sleep(3000);
        AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);

    }

}

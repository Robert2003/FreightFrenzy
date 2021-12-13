package org.firstinspires.ftc.teamcode.autonomous.cases;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.FreightFreznyAuto;

public class Case1 {

    FreightFreznyAuto auto;

    Trajectory traj1;

    public Case1(FreightFreznyAuto auto){
        this.auto = auto;
        traj1 = auto.getMecanumDrive()
                .trajectoryBuilder(auto.getMecanumDrive().getPoseEstimate())
                .lineTo(new Vector2d(14, -24))
                .build();
    }

    public void runAuto(){
        AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 1650);
        while(auto.getRobot().getArmMotor().isBusy());
        auto.getMecanumDrive().followTrajectory(traj1);
        auto.sleep(3000);
        AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
    }

}

package org.firstinspires.ftc.teamcode.autonomous.national;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.detection.FrenzyDetection;
import org.firstinspires.ftc.teamcode.autonomous.national.options.ForcedCase;
import org.firstinspires.ftc.teamcode.autonomous.national.options.Side;

public class SelectionCase {

    FrenzySelection auto;
    Side side;
    ForcedCase forcedCase;

    int armGoTo;

    public SelectionCase(FrenzySelection auto, int armGoTo) {
        this.auto = auto;
        this.armGoTo = armGoTo;
        this.side = side;
        this.forcedCase = forcedCase;
    }

    public void runAuto() {

    }

}

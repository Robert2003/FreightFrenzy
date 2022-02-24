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

    Trajectory deliverPreloadBox,goToCarousel,alignWithCarousel,storageUnitPark;

    int armGoTo;
    int sign;

    public SelectionCase(FrenzySelection auto, int armGoTo) {
        this.auto = auto;
        this.armGoTo = armGoTo;
        this.side = side;
        this.forcedCase = forcedCase;
    }

    private void initializeTrajectories() {
        sign = 1;
        if(side == Side.BLUE) sign = -1;
        deliverPreloadBox = auto.getMecanumDrive()
                .trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(15, sign * (-31)))
                .build();
        goToCarousel = auto.getMecanumDrive()
                .trajectoryBuilder(deliverPreloadBox.end())
                .lineToLinearHeading(new Pose2d(5,sign * 20,Math.toRadians(sign * (-90))))
                .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f * sign))
                .build();
        alignWithCarousel = auto.getMecanumDrive()
                .trajectoryBuilder(goToCarousel.end())
                .back(8)
                .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f * sign))
                .build();
    }
    public void runAuto() {

    }

}

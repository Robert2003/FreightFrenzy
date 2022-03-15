package org.firstinspires.ftc.teamcode.autonomous.national;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.national.option.ForcedCase;
import org.firstinspires.ftc.teamcode.autonomous.national.option.Side;
import org.firstinspires.ftc.teamcode.autonomous.national.option.TeamCompatible;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class SelectionCase {

    FrenzySelection auto;
    Side side;
    ForcedCase forcedCase;
    TeamCompatible teamCompatible;

    Trajectory deliverPreloadBox,goToCarousel,alignWithCarousel,storageUnitPark,alignWithWall,enterWarehouse,exitWarehouse,goToShippingHub,alignWithWall2,strafeToPark,park;
    TrajectorySequence parkingSoft,goToPark,goToWare,getCube;

    int armGoTo;
    int sign;

    public SelectionCase(FrenzySelection auto, int armGoTo) {
        this.auto = auto;
        this.armGoTo = armGoTo;
        this.side = auto.getSide();
        this.forcedCase = auto.getForcedCase();
        this.teamCompatible = auto.getTeamCompatible();

        initializeTrajectories();
    }

    private void initializeTrajectories() {
        sign = 1;
        if(side == Side.BLUE) sign = -1;
        if(teamCompatible == TeamCompatible.SOFT_DELTA) {
            deliverPreloadBox = auto.getMecanumDrive()
                    .trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d( 18, sign * 25))
                    .build();
            alignWithWall = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToSplineHeading(new Pose2d(-26,sign * 0, Math.toRadians(sign * (-150))))
                    .addTemporalMarker(.5, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),15))
                    .build(); // era lineToLinearHeading x: -25 y: 0 h: -135
            enterWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall.end())
                    .forward(20)
                    .build();
            exitWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(enterWarehouse.end())
                    .back(30)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1825))
                    .build();
            goToShippingHub = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse.end())
                    .lineTo(new Vector2d(30,sign * -5))
                    .addTemporalMarker(.3, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),sign*(-1305)))
                    .build();
            alignWithWall2 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub.end())
                    .lineToLinearHeading(new Pose2d(-18,sign * 10,Math.toRadians(sign * (-150))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            parkingSoft = auto.getMecanumDrive().trajectorySequenceBuilder(alignWithWall2.end())
                    .forward(25)
                    .strafeLeft(sign * 32)
                    .forward(18)
                    .turn(Math.toRadians(sign * -135))
                    .build();
        } else if(teamCompatible == TeamCompatible.BAG_UN_CARRY) {
            deliverPreloadBox = auto.getMecanumDrive()
                    .trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d(18, sign * 25))
                    .build();
            alignWithWall = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToSplineHeading(new Pose2d(-26, sign * 0, Math.toRadians(sign * (-150))))
                    .addTemporalMarker(.5, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 15))
                    .build(); // era lineToLinearHeading x: -25 y: 0 h: -135
            enterWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall.end())
                    .forward(20)
                    .build();
            exitWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(enterWarehouse.end())
                    .back(30)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(), 1825))
                    .build();
            goToShippingHub = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse.end())
                    .lineTo(new Vector2d(30, sign * -5))
                    .addTemporalMarker(.3, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(), sign * (-1305)))
                    .build();
            alignWithWall2 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub.end())
                    .lineToLinearHeading(new Pose2d(-18, sign * 10, Math.toRadians(sign * (-150))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(), auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(), 0))
                    .build();
            getCube = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(alignWithWall2.end())
                    .turn(Math.toRadians(sign * 15))
                    .forward(20)
                    .build();
            parkingSoft = auto.getMecanumDrive().trajectorySequenceBuilder(alignWithWall2.end())
                    .forward(25)
                    .strafeLeft(sign * 32)
                    .forward(18)
                    .turn(Math.toRadians(sign * -135))
                    .build();
        } else {
            // ROBOT NEAR CAROUSEL
            deliverPreloadBox = auto.getMecanumDrive()
                    .trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d( 21, sign * (-31)))
                    .build();
            goToCarousel = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(10,sign * 20,Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f * sign))
                    .build();
            alignWithCarousel = auto.getMecanumDrive()
                    .trajectoryBuilder(goToCarousel.end())
                    .back(16,
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f * sign))
                    .build();
            park = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithCarousel.end())
                    .lineToLinearHeading(new Pose2d(41, 43, Math.toRadians(sign * (-90))))
                    .build();
            // ROBOT NEAR WAREHOUSE

            // AUTONOM ALLIANCE
        }

    }

    public void runAuto() {
        if(teamCompatible == TeamCompatible.SOFT_DELTA) {
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            //while(auto.getRobot().getArmMotor().isBusy());
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(alignWithWall);
            //auto.sleep(50);
            auto.getMecanumDrive().followTrajectory(enterWarehouse);
            //auto.sleep(100);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), false);
            auto.getMecanumDrive().followTrajectory(exitWarehouse);
            auto.getMecanumDrive().followTrajectory(goToShippingHub);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            //auto.sleep(50);
            auto.getMecanumDrive().followTrajectory(alignWithWall2);
            auto.getMecanumDrive().followTrajectorySequence(parkingSoft);
        } else if(teamCompatible == TeamCompatible.NONE) {
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            //while(auto.getRobot().getArmMotor().isBusy());
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(goToCarousel);
            auto.getMecanumDrive().followTrajectory(alignWithCarousel);
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), auto.getRobot().getZeroArm());
            auto.sleep(3000);
            auto.getMecanumDrive().followTrajectory(park);
            AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), 0);
        }

    }

}
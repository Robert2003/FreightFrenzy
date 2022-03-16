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

    Trajectory exitWarehouse2, alignWithWall3, goToShippingHub2, enterWarehouse3, enterWarehouse2, deliverPreloadBox,goToCarousel,storageUnitPark,alignWithWall,enterWarehouse,exitWarehouse,goToShippingHub,alignWithWall2,strafeToPark,park;
    TrajectorySequence parkingSoft,goToPark,goToWare,getCube,collectDuck,alignWithCarousel,deliverDuck;

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
                    .lineToLinearHeading(new Pose2d( 18, sign * 17, Math.toRadians(sign * (45))))
                    .build();
            alignWithWall = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(-16,sign * 6, Math.toRadians(sign * (-130))))
                    .addTemporalMarker(.5, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),15))
                    .build();
            enterWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall.end())
                    .forward(34)
                    .build();
            exitWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(enterWarehouse.end())
                    .back(30)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1825))
                    .build();
            goToShippingHub = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse.end())
                    .lineToLinearHeading(new Pose2d(14,sign * 15, Math.toRadians(sign * (45))))
                    .build();
            alignWithWall2 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub.end())
                    .lineToLinearHeading(new Pose2d(-18,sign * 10,Math.toRadians(sign * (-130))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            parkingSoft = auto.getMecanumDrive().trajectorySequenceBuilder(alignWithWall2.end())
                    .forward(25)
                    .strafeLeft(sign * 32)
                    .forward(15)
                    .turn(Math.toRadians(sign * -135))
                    .build();
        } else if(teamCompatible == TeamCompatible.BAG_UN_CARRY) {
            deliverPreloadBox = auto.getMecanumDrive()
                    .trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d( 18, sign * 17, Math.toRadians(sign * (45))))
                    .build();
            alignWithWall = auto.getMecanumDrive()
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(-16,sign * 6, Math.toRadians(sign * (-130))))
                    .addTemporalMarker(.5, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),15))
                    .build();
            enterWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall.end())
                    .forward(34)
                    .build();
            exitWarehouse = auto.getMecanumDrive()
                    .trajectoryBuilder(enterWarehouse.end())
                    .back(30)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1825))
                    .build();
            goToShippingHub = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse.end())
                    .lineToLinearHeading(new Pose2d(14,sign * 15, Math.toRadians(sign * (45))))
                    .build();
            alignWithWall2 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub.end())
                    .lineToLinearHeading(new Pose2d(-18,sign * 10,Math.toRadians(sign * (-130))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            enterWarehouse2 = auto.getMecanumDrive()
                    .trajectoryBuilder(alignWithWall2.end())
                    .forward(34)
                    .build();
            exitWarehouse2 = auto.getMecanumDrive()
                    .trajectoryBuilder(enterWarehouse2.end())
                    .back(30)
                    .addTemporalMarker(.3, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(),1825))
                    .build();
            goToShippingHub2 = auto.getMecanumDrive()
                    .trajectoryBuilder(exitWarehouse2.end())
                    .lineToLinearHeading(new Pose2d(14,sign * 15, Math.toRadians(sign * (45))))
                    .build();
            alignWithWall3 = auto.getMecanumDrive()
                    .trajectoryBuilder(goToShippingHub2.end())
                    .lineToLinearHeading(new Pose2d(-18,sign * 10,Math.toRadians(sign * (-130))))
                    .addTemporalMarker(.6, () -> AutoUtil.plateToPosition(auto.getRobot().getArmMotor(),auto.getRobot().getZeroArm()))
                    .addTemporalMarker(.5, () -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0))
                    .build();
            parkingSoft = auto.getMecanumDrive().trajectorySequenceBuilder(alignWithWall3.end())
                    .forward(25)
                    .strafeLeft(sign * 32)
                    .forward(15)
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
                    .lineToLinearHeading(new Pose2d(5,sign * 20,Math.toRadians(sign * (-135))),
                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(.5, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f * sign))
                    .addTemporalMarker(.5, () -> AutoUtil.armToPosition(auto.getRobot().getArmMotor(), auto.getRobot().getZeroArm()))
                    .build();
            alignWithCarousel = auto.getMecanumDrive()
                    .trajectorySequenceBuilder(goToCarousel.end())
                    .back(10,
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), -.65f * sign))
                    .strafeRight(0.5 * sign)
                    .build();
            collectDuck = auto.getMecanumDrive().trajectorySequenceBuilder(alignWithCarousel.end())
                    .addTemporalMarker(() -> AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), 0))
                    .strafeLeft(10)
                    .turn(Math.toRadians(sign * 30))
                    .forward(25)
                    .turn(Math.toRadians(sign * -240))
                    .strafeLeft(sign * 17)
                    .addTemporalMarker(() -> AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),sign * (-280)))
                    .forward(24,
                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .back(0.78)
                    .addTemporalMarker(() -> AutoUtil.setClawOpen(auto.getRobot().getExcavator(),false))
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
        } else if (teamCompatible == TeamCompatible.BAG_UN_CARRY) {
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
            auto.getMecanumDrive().followTrajectory(alignWithWall2);
            auto.getMecanumDrive().followTrajectory(enterWarehouse2);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), false);
            auto.getMecanumDrive().followTrajectory(exitWarehouse2);
            auto.getMecanumDrive().followTrajectory(goToShippingHub2);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(alignWithWall3);
            auto.getMecanumDrive().followTrajectory(enterWarehouse3);
            auto.getMecanumDrive().followTrajectorySequence(parkingSoft);
        } else if(teamCompatible == TeamCompatible.NONE) {
            AutoUtil.armToPosition(auto.getRobot().getArmMotor(), armGoTo);
            //while(auto.getRobot().getArmMotor().isBusy());
            auto.getMecanumDrive().followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(auto.getRobot().getExcavator(), true);
            auto.getMecanumDrive().followTrajectory(goToCarousel);
            auto.getMecanumDrive().followTrajectorySequence(alignWithCarousel);
            auto.sleep(3000);
            auto.getMecanumDrive().followTrajectorySequence(collectDuck);
            //auto.getMecanumDrive().followTrajectory(park);
            AutoUtil.rotateDucks(auto.getRobot().getFlyWheel(), 0);
            AutoUtil.plateToPosition(auto.getRobot().getPlateMotor(),0);
        }

    }

}
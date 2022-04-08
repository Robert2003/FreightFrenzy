package org.firstinspires.ftc.teamcode.autonomous.national;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.national.option.ForcedCase;
import org.firstinspires.ftc.teamcode.autonomous.national.option.Side;
import org.firstinspires.ftc.teamcode.autonomous.national.option.TeamCompatible;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class SelectionCaseInter {

    Side side;
    ForcedCase forcedCase;
    TeamCompatible teamCompatible;

    Trajectory alignWithWall3, goToShippingHub2, deliverPreloadBox,goToCarousel,storageUnitPark,alignWithWall,exitWarehouse,goToShippingHub,alignWithWall2,strafeToPark,park,goToShippingHub3,alignWithWall4,goToShippingHub4,alignWithWall5;
    TrajectorySequence parkingSoft,goToPark,exitWarehouse2, enterWarehouse3,goToWare,getCube,collectDuck,enterWarehouse,alignWithCarousel,deliverDuck,enterWarehouse2,exitWarehouse3,enterWarehouse4,exitWarehouse4;

    int armGoTo;
    int sign;

    SampleMecanumDrive drive;
    RobotDefinition robot;
    LinearOpMode opmode;

    public SelectionCaseInter(LinearOpMode opmode, SampleMecanumDrive mecanumDrive, RobotDefinition robotDefinition,
                              Side side, TeamCompatible teamCompatible, int armGoTo) {
        this.armGoTo = armGoTo;
        this.side = side;
        this.forcedCase = ForcedCase.DETECTION;
        this.teamCompatible = teamCompatible;
        this.drive = mecanumDrive;
        this.robot = robotDefinition;
        this.opmode = opmode;

        initializeTrajectories();
    }

    private void initializeTrajectories() {
        sign = 1;
        if(side == Side.BLUE) sign = -1;
        if(teamCompatible == TeamCompatible.BAG_UN_CARRY) {
            deliverPreloadBox = drive
                    .trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d( 18, sign * 27))
                    .build();
            enterWarehouse = drive
                    .trajectorySequenceBuilder(deliverPreloadBox.end())
                    .splineToLinearHeading(new Pose2d(-4, sign * -6.7, Math.toRadians(-90)), 0)
                    .build();

        } else if(teamCompatible == TeamCompatible.NONE) {
            deliverPreloadBox = drive
                    .trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d( 18, sign * (-27)))
                    .build();
            goToCarousel = drive
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(0,sign * 15,Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.5, () -> AutoUtil.armToPosition(robot.getArmMotor(), robot.getZeroArm()))
                    .build();
            alignWithCarousel = drive
                    .trajectorySequenceBuilder(goToCarousel.end())
                    .back(7,
                            SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addTemporalMarker(.3, () -> AutoUtil.rotateDucks(robot.getFlyWheel(), -.65f * sign))
                    .strafeRight(sign * 2.78)
                    .build();
            park = drive
                    .trajectoryBuilder(alignWithCarousel.end())
                    .lineToLinearHeading(new Pose2d(28, sign * 24, Math.toRadians(sign * (-90))))
                    .build();
        } else if(teamCompatible == TeamCompatible.IAU_UN_CARRY){
            deliverPreloadBox = drive
                    .trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d( 18, sign * 23, Math.toRadians(sign * (10))))
                    .build();
            alignWithWall = drive
                    .trajectoryBuilder(deliverPreloadBox.end())
                    .lineToLinearHeading(new Pose2d(-3,sign * -2, Math.toRadians(sign * (-90))))
                    .addTemporalMarker(.37, () -> AutoUtil.armToPosition(robot.getArmMotor(),1100))
                    .addTemporalMarker(1.8, () -> AutoUtil.armToPosition(robot.getArmMotor(),robot.getZeroArm()))
                    .build();
            enterWarehouse = drive
                    .trajectorySequenceBuilder(alignWithWall.end())
                    .forward(25)
                    .build();
            parkingSoft = drive
                    .trajectorySequenceBuilder(enterWarehouse.end())
                    .strafeLeft(sign * 30)
                    .build();
        }

    }

    public void runAuto() {
        if (teamCompatible == TeamCompatible.BAG_UN_CARRY) {
            AutoUtil.armToPosition(robot.getArmMotor(),armGoTo);
            drive.followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(robot, robot.getExcavator(), true);
            drive.followTrajectorySequence(enterWarehouse);
        } else if(teamCompatible == TeamCompatible.NONE) {
            opmode.sleep(2000);
            AutoUtil.armToPosition(robot.getArmMotor(), armGoTo);
            //while(robot.getArmMotor().isBusy());
            drive.followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(robot, robot.getExcavator(), true);
            opmode.sleep(500);
            drive.followTrajectory(goToCarousel);
            drive.followTrajectorySequence(alignWithCarousel);
            opmode.sleep(6000);
            drive.followTrajectory(park);
            AutoUtil.rotateDucks(robot.getFlyWheel(), 0);
        } else if(teamCompatible == TeamCompatible.IAU_UN_CARRY){
            opmode.sleep(3000);
            AutoUtil.armToPosition(robot.getArmMotor(), armGoTo);
            drive.followTrajectory(deliverPreloadBox);
            AutoUtil.setClawOpen(robot, robot.getExcavator(), true);
            opmode.sleep(500);
            drive.followTrajectory(alignWithWall);
            drive.followTrajectorySequence(enterWarehouse);
            drive.followTrajectorySequence(parkingSoft);
        }
    }

}
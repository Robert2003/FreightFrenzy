package org.firstinspires.ftc.teamcode.autonomous.national;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.national.options.ForcedCase;
import org.firstinspires.ftc.teamcode.autonomous.national.options.Side;
import org.firstinspires.ftc.teamcode.drive.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class FrenzySelection extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    Side side = Side.RED;
    ForcedCase forcedCase = ForcedCase.DETECTION;

    RobotDefinition robot;
    SampleMecanumDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        runtime.reset();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

        }
    }

    private void initialize(){
        robot = new RobotDefinition(hardwareMap);
        robot = new RobotDefinition(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClawOpen(robot.getExcavator(), true);
    }

    int cursorOption = 1;
    int navigatingDelay = 1500;
    /*
        LISTA OPTIUNI (IDs)
        1 - Parte
        2 - Caz fortat
     */

    private void selectOptions(){
        boolean confirmed = false;
        while(!confirmed && !isStopRequested()){
            cycleOptions();
            sleep(navigatingDelay);
        }
    }

    private void cycleOptions(){
        if(gamepad1.dpad_down){
            cursorOption++;
            if(cursorOption == 3)
                cursorOption = 1;
        }
        if(gamepad1.dpad_right){
            switch(cursorOption){
                case 1:
                    int newSide = side.ordinal() + 1;
                    if(newSide == Side.values().length)
                        newSide = 0;
                    side = Side.values()[newSide];
                    break;
                case 2:
                    int newCase = forcedCase.ordinal() + 1;
                    if(newCase == Side.values().length)
                        newCase = 0;
                    side = Side.values()[newCase];
                    break;
            }
        }
    }

    private void showcaseOptions(){
        telemetry.addData("Side", side.toString() + (cursorOption == 1 ? " <-" : ""));
        telemetry.addData("Forced Case", forcedCase.toString() + (cursorOption == 2 ? " <-" : ""));
        telemetry.addData("", "---------------");
        telemetry.addData("Cycle vertical", "DPAD DOWN");
        telemetry.addData("Cycle horizontal", "DPAD RIGHT");
        telemetry.update();
    }

    public SampleMecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public RobotDefinition getRobot() {
        return robot;
    }

}

package org.firstinspires.ftc.teamcode.autonomous.national;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.national.options.ForcedCase;
import org.firstinspires.ftc.teamcode.autonomous.national.options.Side;

public class FrenzySelection extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    Side side = Side.RED;
    ForcedCase forcedCase = ForcedCase.DETECTION;

    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

        }
    }

    int cursorOption = 1;
    /*
        LISTA OPTIUNI (IDs)
        1 - Parte
        2 - Caz fortat
     */

    private void selectOptions(){
        boolean confirmed = false;
        while(!confirmed && !isStopRequested()){

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

}

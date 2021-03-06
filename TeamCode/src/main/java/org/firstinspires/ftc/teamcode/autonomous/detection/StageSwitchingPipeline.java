package org.firstinspires.ftc.teamcode.autonomous.detection;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class StageSwitchingPipeline extends OpenCvPipeline {

    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat all = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    enum Stage
    {//color difference. greyscale
        detection,//includes outlines
        THRESHOLD,//b&w
        RAW_IMAGE,//displays raw view
    }

    private Stage stageToRenderToViewport = Stage.detection;
    private Stage[] stages = Stage.values();

    //copiat
    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    public static int valMid = 0;
    public static int valLeft = 0;
    public static int valRight = 0;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    public static double offsetX = 0.0/8.0;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    public static double offsetY = 0.0/8.0;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    //private float[] midPos = {4.5f/8f+offsetX, 3.4f/8f+offsetY};//0 = col, 1 = row
    //private float[] leftPos = {4.5f/8f+offsetX, 1.4f/8f+offsetY};
    //private float[] rightPos = {4.5f/8f+offsetX, 5.2f/8f+offsetY};

    public static double[] midPos = {4/8.0+offsetX, 5.5/8.0+offsetY};//0 = col, 1 = row
    public static double[] leftPos = {2/8.0+offsetX, 5.5/8.0+offsetY};
    public static double[] rightPos = {6/8.0+offsetX, 5.5/8.0+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor
    /**
     -Deci pentru midPos, leftPos, rightPos, prima valoare, care e diferita la toate 3 inseamna
     cat de in dreapta sau in stanga sunt dreptunghiurile. Valoare = 0.5, dreptunghiul e in mijloc pe OX,
     daca e mai mare e mai la dreapta si daca e mai mica e mai la stanga

     -Deci pentru midPos, leftPos, rightPos, a doua valoare, care e la fel la toate 3 inseamna
     cat de sus sau jos sunt dreptunghiurile. Valoare = 0.5, dreptunghiul e in mijloc pe OY,
     daca e mai mare e mai jos si daca e mai mica e mai sus
     */

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();
        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */

        //color diff cb.
        //lower cb = more blue = skystone = white
        //higher cb = less blue = yellow stone = grey
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

        //b&w
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 50, 255, Imgproc.THRESH_BINARY_INV);

        //outline/contour
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        yCbCrChan2Mat.copyTo(all);//copies mat object
        //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


        //get values from frame
        double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
        valMid = (int)pixMid[0];

        double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
        valLeft = (int)pixLeft[0];

        double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
        valRight = (int)pixRight[0];

        //create three points
        Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
        Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
        Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

        //draw circles on those points
        Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
        Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
        Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

        //draw 3 rectangles
        Imgproc.rectangle(//1-3
                all,
                new Point(
                        input.cols()*(leftPos[0]-rectWidth/2),
                        input.rows()*(leftPos[1]-rectHeight/2)),
                new Point(
                        input.cols()*(leftPos[0]+rectWidth/2),
                        input.rows()*(leftPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(//3-5
                all,
                new Point(
                        input.cols()*(midPos[0]-rectWidth/2),
                        input.rows()*(midPos[1]-rectHeight/2)),
                new Point(
                        input.cols()*(midPos[0]+rectWidth/2),
                        input.rows()*(midPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(//5-7
                all,
                new Point(
                        input.cols()*(rightPos[0]-rectWidth/2),
                        input.rows()*(rightPos[1]-rectHeight/2)),
                new Point(
                        input.cols()*(rightPos[0]+rectWidth/2),
                        input.rows()*(rightPos[1]+rectHeight/2)),
                new Scalar(0, 255, 0), 3);

        switch (stageToRenderToViewport)
        {
            case THRESHOLD:
            {
                return thresholdMat;
            }

            case detection:
            {
                return all;
            }

            default:
            {
                return input;
            }
        }
    }
}
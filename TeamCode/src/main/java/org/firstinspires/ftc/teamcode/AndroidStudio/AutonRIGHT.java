/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.AndroidStudio;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AndroidStudio.vision.ScanAprilTagPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Auton RIGHT 1 Cone", group = "Production")
public class AutonRIGHT extends LinearOpMode
{
    OpenCvCamera camera;
    ScanAprilTagPipeline scanAprilTagPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int[] stackEncoderLevels = new int[]{0, -150, -250, -350, -581};
    AprilTagDetection tagOfInterest = null;

    ShivaRobotAS robot = new ShivaRobotAS();
    DriveTrainAS driveTrain = new DriveTrainAS();
    SlidesAndGripAS slidesAndGripAS = new SlidesAndGripAS();
    GyroAS gyro = new GyroAS();

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(telemetry, hardwareMap);
        gyro.init(robot);
        slidesAndGripAS.init(robot);
        driveTrain.init(robot, gyro);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        scanAprilTagPipeline = new ScanAprilTagPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(scanAprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = scanAprilTagPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == RIGHT || tag.id == MIDDLE || tag.id == LEFT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        getInPosition();
        dropCone();
        if(tagOfInterest == null){
            park(MIDDLE);
        }
        else{
            park(tagOfInterest.id);
        }
    }

    void getInPosition() throws InterruptedException{
        slidesAndGripAS.closeGrip();
        Thread.sleep(250);
        slidesAndGripAS.moveSlides(-200);
        driveTrain.move(4.4, 0.2);
        driveTrain.move(-0.3, 0.2);
    }

    void dropCone() throws InterruptedException {
        driveTrain.turn(-31, 0.3);
        slidesAndGripAS.moveSlides(-3250);
        Thread.sleep(1000);
        driveTrain.move(0.7, 0.1);
        //DROP STUFF
        slidesAndGripAS.moveSlides(-2000);
        Thread.sleep(1000);
        slidesAndGripAS.openGrip();
        driveTrain.move(-0.8, 0.1);
        slidesAndGripAS.moveSlides(0);
        Thread.sleep(1000);
    }

    void pickupCone(int stackLevel) throws InterruptedException{
        driveTrain.turn(-91, 0.3);
        slidesAndGripAS.moveSlides(stackEncoderLevels[stackLevel]);
        driveTrain.move(2.27, 0.2); //INCREASE AT COMPETITION
        //GRAB STUFF
        slidesAndGripAS.closeGrip();
        Thread.sleep(250);
        slidesAndGripAS.moveSlides(-2000);
        Thread.sleep(1000);;
        driveTrain.move(-2.27, 0.2); // INCREASE AT COMPETITION
    }

    void park (int zone){
        slidesAndGripAS.moveSlides(0);
        driveTrain.turn(-5, 0.5);
        if(zone == LEFT){
            driveTrain.strafe(-1.92, 0.5);
        }
        else if(zone == MIDDLE){
        }
        else if(zone == RIGHT){
            driveTrain.strafe(1.92, 0.7);
        }
        driveTrain.move(-0.4, 0.3);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}// 18385-RC
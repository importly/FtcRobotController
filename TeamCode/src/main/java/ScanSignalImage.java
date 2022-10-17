package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Scan Signal Image", group = "Production")
public class ScanSignalImage extends LinearOpMode {
    OpenCvWebcam webcam;
    String currentColor = "Did Not Change";
    double redValue;
    double greenValue;
    double blueValue;
    double hue;
    double saturation;
    double value;
    Scalar lowHSV;
    Scalar highHSV;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new FindColorPipeline());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Color", currentColor);
            telemetry.addData("Red Value: ", redValue);
            telemetry.addData("Blue Value: ", blueValue);
            telemetry.addData("Green Value: ", greenValue);
            telemetry.addData("Hue: ", hue);
            telemetry.addData("Saturation: ", saturation);
            telemetry.addData("Value: ", value);
            telemetry.update();
        }
    }

        class FindColorPipeline extends OpenCvPipeline {
            Mat mat = new Mat();

            @Override
            public Mat processFrame(Mat input) {
                Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
                hue = Core.sumElems(mat).val[0] //FIND OUT HOW TO FIND AREA OF MATRIX;
                saturation = Core.sumElems(mat).val[1];
                value = Core.sumElems(mat).val[2];

                if(redValue > greenValue && redValue > blueValue){
                    currentColor = "Red";
                }
                else if(greenValue > redValue && greenValue > blueValue){
                    currentColor = "blue";
                }
                else if(blueValue > redValue && blueValue > greenValue){
                    currentColor = "green";
                }

                return mat;
            }
        }
    }

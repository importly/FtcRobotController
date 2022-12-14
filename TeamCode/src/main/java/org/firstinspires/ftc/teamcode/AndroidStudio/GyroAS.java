package org.firstinspires.ftc.teamcode.AndroidStudio;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * 1/20/21 - Manav Sanghvi
 * This class provides a high-level interface to the gyro.
 */
public class GyroAS {
    double globalAngle = 0;
    double secondAngle = 0;
    double thirdAngle = 0;
    private Telemetry telemetry = null;
    private int count = 0;
    public boolean quietMode = false;
    
    public boolean isTurningLeft = false;
    
    Orientation angles = new Orientation();

    BNO055IMU imu = null;

    public void init(ShivaRobotAS robot)
    {
        this.telemetry = robot.telemetry;
        this.imu = robot.imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
    }

    // Return current heading of robot relative to where it was pointing at init() time.
    // Range is -180 - 180
    public double getCurrentAngle() {
        // Change the AxesOrder to one of the values listed here to account for the orientation in which the Gyro is mounted on the robot:
        // See: https://first-tech-challenge.github.io/SkyStone/org/firstinspires/ftc/robotcore/external/navigation/AxesOrder.html
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = angles.firstAngle;
        globalAngle *= -1;

        this.displayInfo();
        return globalAngle;
    }
    
    public boolean isClockwise(float compassPoint) {
      return compassPoint > getCurrentAngle();
    }

    private void displayInfo() {
        if (this.telemetry == null || this.quietMode == true)
            return;
            
        this.telemetry.addData("GYRO: First Angle ", angles.firstAngle);
        this.telemetry.addData("GYRO: Second Angle ", angles.secondAngle);
        this.telemetry.addData("GYRO: Third Angle ", angles.thirdAngle);
        this.telemetry.addData("GYRO: Global Angle ", globalAngle);
        this.telemetry.update();
    }
}

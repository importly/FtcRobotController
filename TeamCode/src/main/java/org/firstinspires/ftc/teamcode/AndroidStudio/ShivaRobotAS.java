package org.firstinspires.ftc.teamcode.AndroidStudio;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is used to define all the specific hardware for our robot.
 * All other code can use this class to refer to the robot hardware as software objects.
 */
public class ShivaRobotAS {

    // Wheel motors
    public DcMotor front_left = null;
    public DcMotor front_right = null;
    public DcMotor back_left = null;
    public DcMotor back_right = null;

    // Dead wheel encoders
    public DcMotor x_encoder = null;
    public DcMotor y_encoder = null;

    public Rev2mDistanceSensor distance_sensor = null;

    // Slides motors
    public DcMotor slides_motor = null;

    //Grip servo
    public Servo grip_servo = null;

    // Gyro
    public BNO055IMU imu = null;

    // Alliance Sensor
    public NormalizedColorSensor allianceColorSensor = null;

    // Constants
    public static final double MOTOR_TICKS_PER_360 = 537.7;
    public static final double DEAD_WHEEL_TICKS = 4190;

    // For doing execution time measurements
    private ElapsedTime period = new ElapsedTime();

    // To allow non OpMode classes to display info to telemetry:
    public Telemetry telemetry;

    /* Constructor */
    public ShivaRobotAS() {}

    /* Initialize standard Hardware interfaces */
    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        // Wheel motors
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_right = hardwareMap.get(DcMotor.class, "front_right");

        slides_motor = hardwareMap.get(DcMotor.class, "slides_motor");

        grip_servo = hardwareMap.get(Servo.class, "grip_servo");

        distance_sensor = hardwareMap.get(Rev2mDistanceSensor.class,"fds");


        // Set Motors to not use encoders
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Reverse the direction of the right side motors,
        // so power with the same sing (+ or -) causes the robot to move in the same direction,
        // no matter which wheels the power is applied to.
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
    }
}

package org.firstinspires.ftc.teamcode.AndroidStudio;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Provides low-level autonomous movement functionality for the robot
 */
public class SlidesAndGripAS {

    private final int MAX_SLIDES_POSITION = 6000;
    private final int MIN_SLIDES_POSITION = 0;

    public DcMotor slides_motor  = null;
    private Servo grip_servo = null;

    public void init(ShivaRobotAS robot)
    {
        slides_motor = robot.slides_motor;
        grip_servo = robot.grip_servo;

        slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grip_servo.setDirection(Servo.Direction.REVERSE);
        grip_servo.scaleRange(0, 0.4);
    }

    public void moveSlides(int position){
        slides_motor.setTargetPosition(position);
        slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides_motor.setPower(0.75);
    }

    public void resetSlides(){
        slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void closeGrip(){
        grip_servo.setPosition(1);
    }

    public void openGrip(){
        grip_servo.setPosition(0);
    }
}

package org.firstinspires.ftc.teamcode.AndroidStudio;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Provides low-level autonomous movement functionality for the robot
 */
public class SlidesAndGripAS {

    private final int MAX_SLIDES_POSITION = 5316;
    private final int MIN_SLIDES_POSITION = 0;

    public DcMotor slides_motor  = null;
    private Servo grip_servo = null;

    public void init(ShivaRobotAS robot)
    {
        slides_motor = robot.slides_motor;
        grip_servo = robot.grip_servo;

        slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grip_servo.scaleRange(0.5, 1);
    }

    public void moveSlides(int position){
        int clampedPos;

        if(position < -MAX_SLIDES_POSITION){
            clampedPos = -MAX_SLIDES_POSITION;
        }
        else if(position > MIN_SLIDES_POSITION){
            clampedPos = MIN_SLIDES_POSITION;
        }
        else{
            clampedPos = position;
        }

        slides_motor.setTargetPosition(clampedPos);
        slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides_motor.setPower(0.75);
    }

    public void closeGrip(){
        grip_servo.setPosition(1);
    }

    public void openGrip(){
        grip_servo.setPosition(0);
    }
}
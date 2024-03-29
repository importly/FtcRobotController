package org.firstinspires.ftc.teamcode.AndroidStudio;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

/**
 * Provides low-level autonomous movement functionality for the robot
 */
public class DriveTrainAS {
    private GyroAS gyro = null;
    private Telemetry telemetry = null;

    // Cache the drivetrain motors in this class
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    private Rev2mDistanceSensor distance_sensor = null;

    private static final float AMPLIFIER = 0.02f;

    public void init(ShivaRobotAS robot, GyroAS newGyro)
    {
        // Save values as instance variables
        gyro = newGyro;
        gyro.quietMode = true;
        telemetry = robot.telemetry;
        front_left  = robot.front_left;
        front_right = robot.front_right;
        back_left   = robot.back_left;
        back_right  = robot.back_right;

        distance_sensor = robot.distance_sensor;

        stop();
        setZeroPowerBehavior();
    }

    // Move the robot forward the specified rotations, at the specified power
    // Power must be a value from -1.0 to 1.0
    // The gyro is used to help the robot drive straight
    public void move(double rotationsToSpin, double power)
    {
        // Get the robot's current heading, and compute the number of ticks needed to move
        float startAngle = (float)gyro.getCurrentAngle();

        // Compute how many ticks we need the dead wheel to spin
        int rotationsInTicks = (int) Math.round(ShivaRobotAS.MOTOR_TICKS_PER_360 * rotationsToSpin);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        setTargetPositions(rotationsInTicks);
        setModes(DcMotor.RunMode.RUN_TO_POSITION);
        startMovement(power);

        // While the robot is moving, use the gyro to help it move in a straight line
        while(front_right.isBusy())
        {
            //adjust(getCorrectionAngle(startAngle), (float)power);
            telemetry.addData("Rotations in Ticks: ", rotationsInTicks);
            telemetry.addData("Encoder Position: ", front_right.getCurrentPosition());
            telemetry.update();
        }
        // Stop robot
        stop();
    }

    // Move the robot sideways to the right the specified rotations, at the specified power
    // Power must be a value from -1.0 to 1.0
    // The gyro is NOT used to help the robot drive straight
    public void strafe(double rotationsToSpin, double power)
    {
        int rotationsInTicks = (int) Math.round(ShivaRobotAS.MOTOR_TICKS_PER_360 * rotationsToSpin);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        setTargetPositions(rotationsInTicks);
        setModes(DcMotor.RunMode.RUN_TO_POSITION);
        startMovement(power);

        // Wait for the robot to reach its destination
        while(front_right.isBusy())
        {
            telemetry.addData("Rotations in Ticks: ", rotationsInTicks);
            telemetry.addData("Encoder Position: ", front_right.getCurrentPosition());
            telemetry.update();
        }

        stop();
    }

    // Turn the robot to the specified compass point, using a spin turn
    // compassPoint must be a value from -180.0 to 180.0
    // power must be a value from -1.0 to 1.0
    public void turn(float compassPoint, double power)
    {
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gyro.isClockwise(compassPoint))
        {
            front_left.setDirection(DcMotorSimple.Direction.REVERSE);
            back_left.setDirection(DcMotorSimple.Direction.REVERSE);
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
            back_right.setDirection(DcMotorSimple.Direction.REVERSE);

            startMovement(power);
            while(gyro.getCurrentAngle() < compassPoint)
            {
                telemetry.addData("Gyro Target Angle: ", compassPoint);
                telemetry.addData("Gyro Current Angle: ", gyro.getCurrentAngle());
                telemetry.update();
            }
        }
        else if (!gyro.isClockwise(compassPoint))
        {
            front_left.setDirection(DcMotorSimple.Direction.FORWARD);
            back_left.setDirection(DcMotorSimple.Direction.FORWARD);
            front_right.setDirection(DcMotorSimple.Direction.FORWARD);
            back_right.setDirection(DcMotorSimple.Direction.FORWARD);

            startMovement(power);
            while(gyro.getCurrentAngle() > compassPoint)
            {
                telemetry.addData("Gyro Target Angle: ", compassPoint);
                telemetry.addData("Gyro Current Angle: ", gyro.getCurrentAngle());
                telemetry.update();
            }
        }

        stop();
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnWithDistanceSensor(double distanceToLookFor, double power, int direction)
    {
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(direction == 1) //Clockwise
        {
            front_left.setDirection(DcMotorSimple.Direction.REVERSE);
            back_left.setDirection(DcMotorSimple.Direction.REVERSE);
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
            back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if (direction == 2) //Counter Clockwise
        {
            front_left.setDirection(DcMotorSimple.Direction.FORWARD);
            back_left.setDirection(DcMotorSimple.Direction.FORWARD);
            front_right.setDirection(DcMotorSimple.Direction.FORWARD);
            back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        startMovement(power);
        while(distance_sensor.getDistance(DistanceUnit.CM) > distanceToLookFor)
        {
            telemetry.addData("Distance To Look Form", distanceToLookFor + " cm");
            telemetry.addData("Current Distance Seen", distance_sensor.getDistance(DistanceUnit.CM) + " cm");
            telemetry.update();
        }

        stop();
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnWithDistanceSensorDelta(float maximumDegrees, double power, int direction)
    {
        float startingAngle = (float)gyro.getCurrentAngle();
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(direction == 1) //Clockwise
        {
            front_left.setDirection(DcMotorSimple.Direction.REVERSE);
            back_left.setDirection(DcMotorSimple.Direction.REVERSE);
            front_right.setDirection(DcMotorSimple.Direction.REVERSE);
            back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if (direction == 2) //Counter Clockwise
        {
            front_left.setDirection(DcMotorSimple.Direction.FORWARD);
            back_left.setDirection(DcMotorSimple.Direction.FORWARD);
            front_right.setDirection(DcMotorSimple.Direction.FORWARD);
            back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        double[] distances = new double[(int)maximumDegrees/2];
        float[] degrees = new float[(int)maximumDegrees/2];
        distances[0] = distance_sensor.getDistance(DistanceUnit.CM);
        degrees[0] = startingAngle;

        startMovement(power);
        int i = 1;

        while (Math.abs(startingAngle - gyro.getCurrentAngle()) <= maximumDegrees){
            if(Math.abs(degrees[i-1] - gyro.getCurrentAngle()) > 2){
                distances[i] = distance_sensor.getDistance(DistanceUnit.CM);
                degrees[i] = (float)gyro.getCurrentAngle();
                i++;
            }
            else{
            }
            telemetry.addData("Current Index", i);
            telemetry.addData("Difference", Math.abs(degrees[i-1] - gyro.getCurrentAngle()));
            telemetry.addData("Current Distance", distances[i]);
            telemetry.addData("Degrees", degrees[i]);
            telemetry.update();
        }

        stop();
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        turn((float)degrees[indexOfSmallest(distances)], 0.3);
    }

    public void moveWithDistanceSensor(double distanceToLookFor, double power)
    {
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        startMovement(power);
        while(distance_sensor.getDistance(DistanceUnit.CM) > distanceToLookFor)
        {
            telemetry.addData("Distance To Look Form", distanceToLookFor + " cm");
            telemetry.addData("Current Distance Seen", distance_sensor.getDistance(DistanceUnit.CM) + " cm");
            telemetry.update();
        }

        stop();
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeWithDistanceSensor(double distanceToLookFor, double power)
    {
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        startMovement(power);
        while(distance_sensor.getDistance(DistanceUnit.CM) > distanceToLookFor)
        {
            telemetry.addData("Distance To Look Form", distanceToLookFor + " cm");
            telemetry.addData("Current Distance Seen", distance_sensor.getDistance(DistanceUnit.CM) + " cm");
            telemetry.update();
        }

        stop();
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop()
    {
        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //telemetry.addData("Status: ", "Stopped");
        //telemetry.update();
    }

    // Move the robot the specified rotations, at the specified power
    // Power must be a value from -1.0 to 1.0
    // The direction to move must be set before calling this method
    private void startMovement(double power) {
        // Start moving the robot
        front_left.setPower((float)power);
        back_left.setPower((float)power);
        front_right.setPower((float)power);
        back_right.setPower((float)power);
    }

    // Adjust the power to the wheels, based on the current gyro reading, in case the robot drifts off course
    private void adjust(float angle, float basePower)
    {
        float powerChange = Math.abs(angle * AMPLIFIER);

        if(angle > 0)
        {
            front_left.setPower(basePower + powerChange);
            back_left.setPower(basePower + powerChange);
            front_right.setPower(basePower - powerChange);
            back_right.setPower(basePower - powerChange);
        }
        else if(angle < 0)
        {
            front_left.setPower(basePower - powerChange);
            back_left.setPower(basePower - powerChange);
            front_right.setPower(basePower + powerChange);
            back_right.setPower(basePower + powerChange);
        }
    }

    public float getCorrectionAngle(float startAngle)
    {
        float correctionAngle = startAngle - (float)gyro.getCurrentAngle();
        return correctionAngle;
    }

    private void setZeroPowerBehavior() {
        DcMotor.ZeroPowerBehavior zpb = DcMotor.ZeroPowerBehavior.BRAKE;
        front_left.setZeroPowerBehavior(zpb);
        back_left.setZeroPowerBehavior(zpb);
        front_right.setZeroPowerBehavior(zpb);
        back_right.setZeroPowerBehavior(zpb);
    }

    private void setModes(DcMotor.RunMode runMode){
        front_left.setMode(runMode);
        back_left.setMode(runMode);
        front_right.setMode(runMode);
        back_right.setMode(runMode);
    }

    private void setTargetPositions(int targetPosition){
        front_left.setTargetPosition(targetPosition);
        back_left.setTargetPosition(targetPosition);
        front_right.setTargetPosition(targetPosition);
        back_right.setTargetPosition(targetPosition);
    }
    public static int indexOfSmallest(double[] array){
        int index = 0;
        double min = array[index];

        for (int i = 1; i < array.length; i++){
            if (array[i] <= min){
                min = array[i];
                index = i;
            }
        }
        return index;
    }
}
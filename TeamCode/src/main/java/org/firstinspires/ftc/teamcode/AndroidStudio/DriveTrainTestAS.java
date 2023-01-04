package org.firstinspires.ftc.teamcode.AndroidStudio;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This op-mode tests the Alliance classes
 */
@Autonomous(name = "DriveTrain Test ANDROID STUDIO", group = "Test")
public class DriveTrainTestAS extends LinearOpMode {
//
  ShivaRobotAS robot = new ShivaRobotAS();
  GyroAS gyro = new GyroAS();
  DriveTrainAS drivetrain = new DriveTrainAS();

  /**
   * Repeatedly test and report the alliance detected by the ShivaAlliance class
   */
  @Override
  public void runOpMode() throws InterruptedException{
    // Initialize the robot interface
    robot.init(telemetry, hardwareMap);
    gyro.init(robot);
    drivetrain.init(robot, gyro);

    // Wait for the start button to be pressed.
    //waitForStart();
//
//    // Drive the robot a short distance forward, backward, left, and right, and then turn 180 in each direction.
    //drivetrain.move(1, 0.3);
//    sleep(1000);
//    drivetrain.move(-1, 0.3);
//    sleep(1000);
//    drivetrain.strafe(1, 0.3);
//    sleep(1000);
//    drivetrain.strafe(-1, 0.3);
//    sleep(1000);
//    drivetrain.turn(90, 0.3);
//    sleep(1000);
//    drivetrain.turn(-90, 0.3);

    Rev2mDistanceSensor distance_sensor_right = hardwareMap.get(Rev2mDistanceSensor.class,"distance right");
    Rev2mDistanceSensor distance_sensor_left = hardwareMap.get(Rev2mDistanceSensor.class,"distance left");

    while (opModeIsActive()) {
      telemetry.addData(distance_sensor_right.getDeviceName(), distance_sensor_right.getDistance(DistanceUnit.CM));
      telemetry.addData(distance_sensor_left.getDeviceName(), distance_sensor_left.getDistance(DistanceUnit.CM));

      telemetry.update();

    }
  }
}

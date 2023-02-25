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

    while (!isStarted() && !isStopRequested()){
      telemetry.addData("Current Distance: ", robot.distance_sensor.getDistance(DistanceUnit.CM));

      telemetry.update();
    }

    drivetrain.turnWithDistanceSensor(25, 0.5, 1);

    while (opModeIsActive()) {
      telemetry.addData(robot.distance_sensor.getDeviceName(), robot.distance_sensor.getDistance(DistanceUnit.CM));

      telemetry.update();
    }
  }
}

package org.firstinspires.ftc.teamcode.AndroidStudio;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This op-mode tests the Alliance classes
 */
@Autonomous(name = "DriveTrain Test ANDROID STUDIO", group = "Test")
public class DriveTrainTestAS extends LinearOpMode {

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
    waitForStart();

    // Drive the robot a short distance forward, backward, left, and right, and then turn 180 in each direction.
    drivetrain.move(1, 0.3);
    sleep(1000);
    drivetrain.move(-1, 0.3);
    sleep(1000);
    drivetrain.strafe(1, 0.3);
    sleep(1000);
    drivetrain.strafe(-1, 0.3);
    sleep(1000);
    drivetrain.turn(90, 0.3);
    sleep(1000);
    drivetrain.turn(-90, 0.3);

    while (opModeIsActive()) {

    }
  }
}

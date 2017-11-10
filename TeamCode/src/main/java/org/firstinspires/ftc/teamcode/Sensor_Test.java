package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.SENSORTEST;

/**
 * Created by FTC8424 on 1/14/2017.
 *
 * Autonomous blue, in the front (nearest audience)
 *
 */
@Autonomous(name = "Sensor Test", group = "Test")
public class Sensor_Test extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(SENSORTEST);
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);

        /*int blueValue = 0;
        int redValue = 0;
        double colorTimer = 0;
        */
        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        int heading = 0;
        double driveSpeed = robot.DRIVE_SPEED; waitForStart();
        robot.encoderDrive( this, driveSpeed, 4, 4, 4);
        sleep(1000);
        robot.encoderDrive( this, driveSpeed, 5, 9, 4);
        robot.encoderDrive( this, driveSpeed, 4.9, 2, 4);
        sleep(1000);
        robot.encoderDrive( this, driveSpeed, 4, 4, 4);
          }
}

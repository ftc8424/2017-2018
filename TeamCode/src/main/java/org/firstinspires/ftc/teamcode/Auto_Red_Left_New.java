package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.COLORTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/14/2017.
 */
@Autonomous(name = "Auto Red Left New", group = "RedSide")
public class Auto_Red_Left_New extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(COLORTEST);
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);

        int blueValue = 0;
        int redValue = 0;

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double driveSpeed = .5;
        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        robot.color.enableLed(true);
        sleep(1000L);
        robot.color.enableLed(false);
        waitForStart();
//deploy arm here
            robot.color.enableLed(true);
        sleep(10);
        blueValue = robot.color.blue();
        redValue = robot.color.red();
        telemetry.addData("ColorValue:", "Blue-%d Red-%d", blueValue , redValue);
        telemetry.update();
        sleep(5000);

        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
            telemetry.addData("Color", "blue");
            telemetry.update();
            //robot.encoderDrive(this,0.25,-0.5,0.5,2);
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            telemetry.addData("Color", "red");
            telemetry.update();
            //robot.encoderDrive(this,0.25,0.5,-0.5,2);
        } else {
            telemetry.addData("Color", "cant detect color");
            telemetry.update();

        }
          robot.color.enableLed(false);


        sleep(1000);
        if ( !opModeIsActive() ) return;



//        robot.encoderDrive(this, driveSpeed, 2, 2, 5);
//
//        robot.leftPush.setPosition(robot.lpushStart);
//        robot.rightPush.setPosition(robot.rpushStart);
//
//
//        robot.encoderDrive(this, driveSpeed, -10, -10, 10);
//        //Turning right towards beacon 2
//        robot.encoderDrive(this, 0.25, 12, -12, 10);
//        //Driving towards beacon 2
//        robot.encoderDrive(this, driveSpeed, 47.5, 47.5, 10);
//        //Turning left at Beacon 2
//        robot.encoderDrive(this, 0.25, -12.5, 12.5, 10);
//        //Moving forward to get close enough to hit the beacon
//        robot.encoderDrive(this, driveSpeed, 3.5, 3.5, 10);
//
//        button = "Not Pressing";
//        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
//            robot.rightPush.setPosition(robot.rpushDeploy);
//            button = "Right/Blue";
//        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
//            robot.leftPush.setPosition(robot.lpushDeploy);
//            button = "Left/Red";
//        } else {
//            button = String.format("Not Pressing: Blue %d / Red %d",
//                    robot.color.blue(), robot.color.red());
//        }
//        telemetry.addData("Pressing: %s", button);
//        telemetry.update();
//        sleep(1000);
//        if ( !opModeIsActive() ) return;
//
//        //Resetting button pushers to starting position
//
//
//        robot.encoderDrive(this, driveSpeed, 1, 1, 10);
//
//        robot.leftPush.setPosition(robot.lpushStart);
//        robot.rightPush.setPosition(robot.rpushStart);
//
//        //backing up from beacon 2
//        robot.encoderDrive(this, driveSpeed, -10.5, -10.5, 8);
//        //aligning (turning) to shoot
//        robot.encoderDrive(this, driveSpeed, 6.5, -6.5, 5);
//        //shooting
//        robot.encoderDrive(this, driveSpeed, -31.5, -31.5, 10);
//
//        robot.autoLauncher(this, 0.65);
//        //forward to capball and parking on center
//        robot.encoderDrive(this, driveSpeed, -14, -14, 10);


    }
}

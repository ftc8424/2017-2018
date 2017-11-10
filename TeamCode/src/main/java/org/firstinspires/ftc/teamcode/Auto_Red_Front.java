package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.COLORTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/14/2017.
 *
 * Autonomous red, in the front (nearest audience)
 *
 */
@Autonomous(name = "Auto Red Front", group = "RedSide")
public class Auto_Red_Front extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);

        int blueValue = 0;
        int redValue = 0;

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double driveSpeed = robot.DRIVE_SPEED;
        int heading = 0;
        robot.color.enableLed(true);
        sleep(1000L);
        robot.color.enableLed(false);
        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        robot.gyro.calibrate();
        while(!isStopRequested() && robot.gyro.isCalibrating()){
            telemetry.addData("Init:", "Calibrating");
            telemetry.update();
        }
        while ( !isStarted() ) {
            robot.gyro.resetZAxisIntegrator();
            heading = robot.gyro.getHeading();
            telemetry.addData("Init:", "Calibrated!!");
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }
        robot.deploy(robot.colorArm);

        sleep(1500);
        if ( !opModeIsActive() ) return;
        robot.color.enableLed(true);
        double turnInch = 2;
        double colorTimer = runtime.milliseconds();

        int times = 0;

        do {
            sleep(10);
            blueValue = robot.color.blue();
            redValue = robot.color.red();
            telemetry.addData("color blue", blueValue);
            telemetry.addData("color red", redValue);
            telemetry.update();

            if ( times > 0)
            {
                robot.colorArmAdjust();
                if( times % 3 == 0)
                {
                    robot.encoderDrive(this, 0.25, 0.5, -0.5 ,1);

                }
            }
            times++;
        }while ( opModeIsActive() && runtime.milliseconds() < colorTimer+10000  && (Math.abs(blueValue-redValue) == 0));
        robot.color.enableLed(false);



        if ( blueValue > redValue ) {
            telemetry.addData("Color", "blue");
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            robot.encoderDrive(this, 0.75, turnInch, -turnInch ,2);
            robot.deploy(robot.colorArm);
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            sleep(100);
            robot.encoderDrive(this, 0.75, -turnInch, turnInch, 2);
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        } else if ( blueValue < redValue ) {
            telemetry.addData("Gyro:", heading);
            telemetry.addData("Color", "red");
            telemetry.update();
            robot.encoderDrive(this, 0.75, -turnInch, turnInch, 2);
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            robot.deploy(robot.colorArm);
            sleep(100);
            robot.encoderDrive(this, 0.75, turnInch, -turnInch, 2);
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        } else {
            telemetry.addData("Color", "cant detect color");
            telemetry.update();
            robot.deploy(robot.colorArm);
            sleep(100);
        }
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 23, 23, 5);
        telemetry.addData("Gyro:", heading);
        telemetry.update();
        robot.gyro.resetZAxisIntegrator();
        sleep(1000);
        heading = robot.gyro.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if (robot.gyroTurn(this, 115, 10) == false) {
            heading = robot.gyro.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.addData("Gyro", "turn unsuccessful");
            telemetry.update();
            this.stop();
        }
        heading = robot.gyro.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 35, 35, 10);
        heading = robot.gyro.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();
        if ( !opModeIsActive() ) return;
        robot.gyroTurn(this, 170, 10);
        heading = robot.gyro.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 18, 18, 10);
        heading = robot.gyro.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        double manipTimer = runtime.milliseconds();
        do {
            robot.leftManip.setPower(0.75);
            robot.rightManip.setPower(0.75);
        } while ( opModeIsActive() && runtime.milliseconds() < manipTimer+2000 );
        robot.encoderDrive(this, robot.DRIVE_SPEED, -4, -4, 10);
        robot.leftManip.setPower(0);
        robot.rightManip.setPower(0);
    }
}

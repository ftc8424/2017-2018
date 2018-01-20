package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        int whichColumn = 0;
        int blueValue = 0;
        int redValue = 0;

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double driveSpeed = robot.DRIVE_SPEED;
        double heading = 0;
        robot.color.enableLed(true);
        sleep(1000L);
        robot.color.enableLed(false);
        telemetry.addData("Init:" ,"Waiting for start");
        telemetry.update();
        robot.gyroCalibrate();
        while(!isStopRequested() && (robot.gyroIsCalibrating() || whichColumn == 0)){



            if(whichColumn == 2) telemetry.addData("You have chosen:", "Center Column");

            else if(whichColumn == 1 ) telemetry.addData("You have chosen:", "Left Column");

            else if( whichColumn == 3) telemetry.addData("You have chosen:", "Right Column");

            else telemetry.addData("Choose a", "Column");

            if (gamepad1.a || gamepad1.y) {

                whichColumn = 2;
            }

            if (gamepad1.x) {

                whichColumn = 1;
            }

            if (gamepad1.b) {

                whichColumn = 3;
            }
            telemetry.addData("Init:", "Calibrating");
            telemetry.update();
        }
        while ( !isStarted() ) {
            //robot.gyro.gyroResetZAxisIntegrator();
            heading = robot.getHeading();
            telemetry.addData("Init:", "Calibrated!!");
            telemetry.addData("Gyro:", heading);


            if(whichColumn == 2) telemetry.addData("Column Selected:", "Center Column");

            else if(whichColumn == 1 ) telemetry.addData("Column Selected:", "Left Column");

            else if( whichColumn == 3) telemetry.addData("Column Selected:", "Right Column");

            else {
                telemetry.addData("Column Selected", "Center (Automatic)");
                whichColumn = 2;
            }

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

            if ( times > 0 && times < 6)
            {
                robot.colorArmAdjust();
                if( times % 3 == 0)
                {
                    robot.encoderDrive(this, 0.25, 0.5, -0.5 ,1);

                }
            }
            times++;

            blueValue = robot.color.blue();
            redValue = robot.color.red();
            telemetry.addData("color blue", blueValue);
            telemetry.addData("color red", redValue);
            telemetry.update();

        } while ( opModeIsActive() && runtime.milliseconds() < colorTimer+10000  && (Math.abs(blueValue-redValue) == 0));
        robot.color.enableLed(false);

        if ( times > 0)
        {
            int moveTimes = times/3;
            robot.encoderDrive(this,0.25,-0.5 * moveTimes,0.5 * moveTimes,1);
        }




        if ( blueValue > redValue ) {
            telemetry.addData("Color", "blue");
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            sleep(1000);
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
            sleep(1000);
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
        robot.encoderDrive(this, driveSpeed, 18, 18, 5);
        telemetry.addData("Gyro:", heading);
        telemetry.update();
        robot.gyroResetZAxisIntegrator();
        sleep(1000);
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if (robot.gyroTurn(this, 115, 10) == false) {
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.addData("Gyro", "turn unsuccessful");
            telemetry.update();
            this.stop();
        }
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();
//38.25 means the left column, 31.5 means the center column, 24.75 means the right column (from driver view)
        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 24.75, 24.75, 10);
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();
        if ( !opModeIsActive() ) return;
        robot.gyroTurn(this, 180, 10);
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 15, 15, 10);
        heading = robot.getHeading();
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/14/2017.
 *
 * Autonomous blue, in the back (farthest audience)
 *
 */
@Autonomous(name = "Auto Red Back", group = "BlueSide")
public class Auto_Red_Back extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap);
        int whichColumn = 0;
        int blueValue = 0;
        int redValue = 0;
        double colorTimer = 0;
        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;
        double heading = 0;
        double driveSpeed = robot.DRIVE_SPEED;
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
            //If the whichColumn variable is 1, it is left, if it is 2, it is center, and if it is 3, it is right.

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
        colorTimer = runtime.milliseconds();


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
            telemetry.update();
            robot.encoderDrive(this, 0.75, turnInch, -turnInch ,2);
            //robot.gyroTurn(this, 360-15, 3);
            robot.deploy(robot.colorArm);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            sleep(100);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            robot.encoderDrive(this, 0.75, -turnInch, turnInch, 2);
            //robot.gyroTurn(this, 0, 3);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        } else if ( blueValue < redValue ) {
            telemetry.addData("Color", "red");
            telemetry.update();
            robot.encoderDrive(this, 0.75, -turnInch, turnInch, 2);
            //robot.gyroTurn(this, 15, 3);
            robot.deploy(robot.colorArm);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            sleep(100);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            robot.encoderDrive(this, 0.75, turnInch, -turnInch, 2);
            //robot.gyroTurn(this, 0, 3);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        } else {
            telemetry.addData("Color", "cant detect color");
            telemetry.update();
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            robot.deploy(robot.colorArm);
            sleep(100);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }

        //This following code will allow the robot to move forward on the balancing stone, which was messing with the
        //gyroTurn, So it will move forward 21 inches, and then turn where it was supposed to.

        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 18, 18, 5);
        telemetry.addData("Gyro:", heading);
        telemetry.update();
        robot.gyroResetZAxisIntegrator();
        sleep(1000);
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();
//TODO make the turn less steep.
        if ( !opModeIsActive() ) return;
        //robot.gyroTurn2(this, robot.TURN_SPEED, 265);
        //110 heading is Right column from outside, 135, is left column, 122.5 is middle
        if ( robot.gyroTurn(this, 135, 10) == false) {
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.addData("Gyro", "turn unsuccessful");
            telemetry.update();
            this.stop();
        }
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 20, 20, 10);
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();
        if ( !opModeIsActive() ) return;
        robot.gyroTurn(this, 90, 10);
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 6, 6, 10);
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

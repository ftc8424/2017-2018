package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.COLORTEST;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.FULLAUTO;

/**
 * Created by FTC8424 on 1/14/2017.
 *
 * Autonomous blue, in the front (nearest audience)
 *
 */
@Autonomous(name = "Auto Blue Front", group = "BlueSide")
public class Auto_Blue_Front extends LinearOpMode {
    HardwareHelper robot = new HardwareHelper(FULLAUTO);
    private ElapsedTime runtime = new ElapsedTime();
    VuforiaLocalizer vuforia;


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
        // TODO - Production comment previous two lines and uncomment next, don't need view
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AaknBtv/////AAAAmfjrebUDlEXxppBL0lNYYmF+OrNdDEb+iCZTJVBlijmEOVgaagv19vhNOnlYuDK9CDBvcAzED7y/GhmxAzxn9GKPz6PMCbATksji1FgAfR7zLxWRJKGUxLSQoFMHfAem/xSKUwuTNOohEXW74qhy+KD6VuCwmZFamYthtv/ChxI4o2lwNI4aJDmmpK4jf2I5gr1ULsWML3+OauayyNp74Xa7MLV0mEY2m3sWjnFiZyoygU06ht4+PY2Z/S9/bJvxSgqVfzzFAHzepMJoKAvH+iuDhdHctpnIyXSBtVwcYdoc4GX1+0VjZUb3LzyNSje3NzvUZ8s/n7crdkujseblOuRh8HdqJYAwGCRQrW9mVt+F\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        // The ACTIVATE needs to go AFTER you press the Start button, not before


        while (!isStopRequested() && (robot.gyroIsCalibrating())) {
            telemetry.addData("Init:", "Calibrating");
            telemetry.update();
        }

        while (!isStarted()) {
            //robot.gyro.gyroResetZAxisIntegrator();
            heading = robot.getHeading();
            telemetry.addData("Init:", "Calibrated!!");
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = null;
        double startTime = runtime.milliseconds();

        do {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        } while ( opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.milliseconds() < startTime+5000 );
        telemetry.addData("Vumark", "%s is visible", vuMark);
        telemetry.update();





        robot.deploy(robot.colorGate);
        sleep(500);
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
            robot.encoderDrive(this, 0.75, -turnInch, turnInch ,2);
            //robot.gyroTurn(this, 360-15, 3);
            robot.deploy(robot.colorArm);
            sleep(500);
            robot.deploy(robot.colorGate);
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
        } else if ( blueValue < redValue ) {
            telemetry.addData("Color", "red");
            telemetry.update();
            robot.encoderDrive(this, 0.75, turnInch, -turnInch, 2);
            //robot.gyroTurn(this, 15, 3);
            robot.deploy(robot.colorArm);
            sleep(500);
            robot.deploy(robot.colorGate);

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
        } else {
            telemetry.addData("Color", "cant detect color");
            telemetry.update();
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            robot.deploy(robot.colorArm);
            sleep(500);
            robot.deploy(robot.colorGate);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }
        //This following code will allow the robot to move forward on the balancing stone, which was messing with the
        //gyroTurn, So it will move forward 21 inches, and then turn where it was supposed to.

        int driveForwardFirst = 31;

        int driveForwardSecond = 15;

        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 19, 19, 5);
        telemetry.addData("Gyro:", heading);
        telemetry.update();
        robot.gyroResetZAxisIntegrator();
        sleep(1000);
        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();

        if ( !opModeIsActive() ) return;
        //robot.gyroTurn2(this, robot.TURN_SPEED, 265);

        heading = robot.getHeading();
        telemetry.addData("Gyro:", heading);
        telemetry.update();
//38.25 means the left column, 31.5 means the center column, 24.75 means the right column (from driver view)
        if ( !opModeIsActive() ) return;

        if(vuMark == RelicRecoveryVuMark.RIGHT) {
            if ( robot.gyroTurn(this, 248, 10) == false) {
                heading = robot.getHeading();
                telemetry.addData("Gyro:", heading);
                telemetry.addData("Gyro", "turn unsuccessful");
                telemetry.update();
                this.stop();
            }driveForwardFirst = 38;
            driveForwardSecond = 14;
            robot.encoderDrive(this, driveSpeed, driveForwardFirst, driveForwardFirst, 10);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            if ( !opModeIsActive() ) return;
            robot.gyroTurn(this, 180, 10);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }
        else if(vuMark == RelicRecoveryVuMark.LEFT) {
            if ( robot.gyroTurn(this, 248, 10) == false) {
                heading = robot.getHeading();
                telemetry.addData("Gyro:", heading);
                telemetry.addData("Gyro", "turn unsuccessful");
                telemetry.update();
                this.stop();
            }driveForwardFirst = 25;
            driveForwardSecond = 15;
            robot.encoderDrive(this, driveSpeed, driveForwardFirst, driveForwardFirst, 10);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            if ( !opModeIsActive() ) return;
            robot.gyroTurn(this, 180, 10);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }
        else{
            if ( robot.gyroTurn(this, 248, 10) == false) {
                heading = robot.getHeading();
                telemetry.addData("Gyro:", heading);
                telemetry.addData("Gyro", "turn unsuccessful");
                telemetry.update();
                this.stop();
            }driveForwardFirst = 32;
            driveForwardSecond = 15;
            robot.encoderDrive(this, driveSpeed, driveForwardFirst, driveForwardFirst, 10);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
            if ( !opModeIsActive() ) return;
            robot.gyroTurn(this, 180, 10);
            heading = robot.getHeading();
            telemetry.addData("Gyro:", heading);
            telemetry.update();
        }


        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, driveForwardSecond, driveForwardSecond, 5);
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

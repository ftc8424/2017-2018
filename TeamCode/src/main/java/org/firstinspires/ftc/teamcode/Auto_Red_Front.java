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
        telemetry.addData("Init:", "Calibrated!!");
        telemetry.update();
        waitForStart();
        //robot.color.enableLed(true);
        robot.deploy(robot.colorArm);

        sleep(1500);
        if ( !opModeIsActive() ) return;
        robot.color.enableLed(true);
        double turnInch = 2;
        double colorTimer = runtime.milliseconds();
        do {
            sleep(10);
            blueValue = robot.color.blue();
            redValue = robot.color.red();
            telemetry.addData("color blue", blueValue);
            telemetry.addData("color red", redValue);
            telemetry.update();
        }while ( opModeIsActive() && runtime.milliseconds() < colorTimer+10000  && (Math.abs(blueValue-redValue) == 0));
        robot.color.enableLed(false);
        if ( blueValue > redValue ) {
            telemetry.addData("Color", "blue");
            telemetry.update();
            robot.encoderDrive(this, 0.75, -turnInch, turnInch ,2);
            robot.deploy(robot.colorArm);
            sleep(100);
            robot.encoderDrive(this, 0.75, turnInch, -turnInch, 2);
        } else if ( blueValue < redValue ) {
            telemetry.addData("Color", "red");
            telemetry.update();
            robot.encoderDrive(this, 0.75, turnInch, -turnInch, 2);
            robot.deploy(robot.colorArm);
            sleep(100);
            robot.encoderDrive(this, 0.75, -turnInch, turnInch, 2);
        } else {
            telemetry.addData("Color", "cant detect color");
            telemetry.update();
            robot.deploy(robot.colorArm);
            sleep(100);
        }

        if ( !opModeIsActive() ) return;

        if (robot.gyroTurn(this, 270, 5) == false) {
            telemetry.addData("Gyro", "turn unsuccessful");
            telemetry.update();
        }
        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 34, 34, 10);
        if ( !opModeIsActive() ) return;
        robot.gyroTurn(this, 180, 5);

        // TODO Complete Red Auto with deliver glyph

    }
}

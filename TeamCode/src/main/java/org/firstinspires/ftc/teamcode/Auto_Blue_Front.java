package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.COLORTEST;

/**
 * Created by FTC8424 on 1/14/2017.
 *
 * Autonomous blue, in the front (nearest audience)
 *
 */
@Autonomous(name = "Auto Blue Front", group = "BlueSide")
public class Auto_Blue_Front extends LinearOpMode {
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

        robot.deploy(robot.colorArm);

        sleep(10);
        blueValue = robot.color.blue();
        redValue = robot.color.red();
        telemetry.addData("ColorValue:", "Blue-%d Red-%d", blueValue , redValue);
        telemetry.update();
        sleep(5000);
        if ( !opModeIsActive() ) return;

        double turnInch = 0.5;
        if (robot.color.blue() > 0 && robot.color.blue() > robot.color.red()) {
            telemetry.addData("Color", "blue");
            telemetry.update();
            robot.encoderDrive(this, 0.25, -turnInch, turnInch ,2);
            robot.encoderDrive(this, 0.25, turnInch, -turnInch, 2);
        } else if (robot.color.red() > 0 && robot.color.red() > robot.color.blue()) {
            telemetry.addData("Color", "red");
            telemetry.update();
            robot.encoderDrive(this, 0.25, turnInch, -turnInch, 2);
            robot.encoderDrive(this, 0.25, -turnInch, turnInch, 2);
        } else {
            telemetry.addData("Color", "cant detect color");
            telemetry.update();
        }
          robot.deploy(robot.colorArm);

        if ( !opModeIsActive() ) return;

        robot.gyroTurn(this, 270, 5);
        if ( !opModeIsActive() ) return;
        robot.encoderDrive(this, driveSpeed, 34, 34, 10);
        if ( !opModeIsActive() ) return;
        robot.gyroTurn(this, 180, 5);

        // TODO Complete Blue Auto with deliver glyph

    }
}

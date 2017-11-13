/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import static org.firstinspires.ftc.teamcode.HardwareHelper.RobotType.REVTROLLBOT;

/**
 * Created by FTC8424 on 9/15/2016.
 */

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 *
 * It's a crab-drive base trollbot.  It has a color sensor and a servo
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Krab TeleOp", group="Iterative Opmode")

public class Krab_Drive extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareHelper robot = new HardwareHelper(REVTROLLBOT);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());

        double rightStickVal = -gamepad1.right_stick_y;
        double leftStickVal = -gamepad1.left_stick_y;
        double rightStickXVal = gamepad1.right_stick_x;
        double leftStickXVal = gamepad1.left_stick_x;

        if ( Math.abs(rightStickXVal) > 0.2 || Math.abs(leftStickXVal) > 0.2 ) {
            robot.sideDrive(this, leftStickXVal, rightStickXVal);
        } else if ( Math.abs(rightStickVal) > 0.01 || Math.abs(leftStickVal) > 0.01 ) {
            robot.normalDrive(this, leftStickVal, rightStickVal);
        } else {
            robot.normalDrive(this, 0.0, 0.0);
        }
        int red = robot.color.red();
        int blue = robot.color.blue();
        int green = robot.color.green();
        telemetry.addData("Color:", "Blue %d, Red %d, Green %d", blue, red, green);
        telemetry.addData("Encoders:", "LF - %d, RF - %d, LB - %d, RB - %d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.leftBackDrive.getCurrentPosition(),
                robot.rightBackDrive.getCurrentPosition());
        telemetry.addData("Gyro Heading:", "%.1f",
                robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    } // loop

    /*,
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.normalDrive(this, 0.0, 0.0);
    }
}

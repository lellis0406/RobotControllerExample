/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
This program contains an example of how to set up an OpMode, and run one motor.
 */

@TeleOp(name="test_1_motor", group="Linear OpMode")
//Used to tell the program how to show up on the driver hub. MUST be present.

//@Disabled
//If you uncomment the line above, this OpMode will not show up on the driverhub.
//Useful for when you have old code that you still want to keep but don't intend
//to use in the competition.
public class OpModeExample extends LinearOpMode {
    private DcMotor leftFrontDrive;

    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "themotor");
        //In the rev driver hub, you have to name all the devices on your robot
        //(including motors). Once you have done this, you
        //need to hardware map these motors
        //using this function. The name in
        //the devicename MUST match the name on the driver hub for each device.
        //You must also provide the device type (see above).

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        //Sometimes, motors will be facing backwards on the robot.
        //This is how you flip its direction at the start rather than having
        //to reverse all of the inputs for the whole program.

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("testing");
        telemetry.update();
        //Telemetry is a very important part of debugging and programming your robot.
        //It allows you to output data to the screen of your driver hub.
        //You MUST call telemetry.update every frame, or it will stop rendering
        //on the driver hub screen.
        //You can have multiple things on the screen at once; just call
        //addData multiple times.

        waitForStart();
        //this pauses the program until the play button is pressed again.


        while (opModeIsActive()) {
            //Everything within this loop will now run until the stop button is pressed.
            double motor_speed =  1.0;

            // Send calculated power to the motor
            leftFrontDrive.setPower(motor_speed);

            telemetry.update();
        }
    }}

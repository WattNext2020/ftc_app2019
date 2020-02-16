/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "Delay12_Depot", group= "Linear Opmode")

public class Delay12_Depot extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfr = null;
    private DcMotor leftback = null;
    private DcMotor rightfr = null;
    private DcMotor rightback = null;

    //private CRServo rackPinionUD = null;
    //private CRServo rackPinionLR = null;

    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;

    double leftPower;
    double rightPower;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftfr = hardwareMap.get(DcMotor.class, "leftf");
        leftback = hardwareMap.get(DcMotor.class, "leftb");
        rightfr = hardwareMap.get(DcMotor.class, "rightf");
        rightback = hardwareMap.get(DcMotor.class, "rightb");


        //rackPinionUD = hardwareMap.get (CRServo.class, "rpUpDown");
        //rackPinionLR = hardwareMap.get (CRServo.class, "rpLeftRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfr.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightfr.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            if(runtime.seconds() < 25) {
                try {
                    Thread.sleep(25000);
                } catch (InterruptedException ie) {

                }
            }
            while (runtime.seconds() < 26.1) {
                leftfr.setPower(0.5);
                leftback.setPower(0.5);
                rightfr.setPower(0.5);
                rightback.setPower(0.5);
            }
            leftfr.setPower(0.0);
            leftback.setPower(0.0);
            rightfr.setPower(0.0);
            rightback.setPower(0.0);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}


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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.math.BigDecimal;
import java.math.RoundingMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import java.lang.Math;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation.AngleSet;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import java.util.Locale;


import java.lang.Math;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name= "RED Foundation AcuTurn Long", group= "Linear Opmode")

public class RED_FoundationAcuTurn extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfr = null;
    private DcMotor leftback = null;
    private DcMotor rightfr = null;
    double initHeading;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU imu;

    private DcMotor rightback = null;

    private CRServo leftWheels = null;
    private CRServo rightWheels = null;
    //private CRServo rackPinionUD = null;
    //private CRServo rackPinionLR = null;

    private Servo rightHook = null;
    private Servo leftHook = null;

    private DistanceSensor sensorRange;

    Servo newCap;


    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;

    double leftPower;
    double rightPower;
    double rackPowerUD;
    double rackPowerLR;


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

        leftWheels = hardwareMap.get(CRServo.class, "lw");   //leftwheels
        rightWheels = hardwareMap.get(CRServo.class, "rw");  //rightwheels
        // capStone = hardwareMap.get (CRServo.class, "Cap");  // Rack and Pinion Vertical
        // rackPinionLR = hardwareMap.get (CRServo.class, "rpLeftRight");   //Rack and Pinion Horizontal
        rightHook = hardwareMap.get(Servo.class, "rightHook");
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        newCap = hardwareMap.get(Servo.class, "NewCap");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);



        newCap.setPosition(0);



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        leftWheels.setDirection(CRServo.Direction.REVERSE);
        rightWheels.setDirection(CRServo.Direction.FORWARD);
        //rackPinionUD.setDirection(CRServo.Direction.FORWARD);
        //rackPinionLR.setDirection(CRServo.Direction.FORWARD);

        rightHook.setDirection(Servo.Direction.REVERSE);
        //rightfr.setmode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftfr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //int MOTORTICKS = 1680;


        leftHook.setPosition(0);
        rightHook.setPosition(0);

        leftWheels.setPower(0);
        rightWheels.setPower(0);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            while (runtime.seconds() < 1) {
                leftfr.setPower(-0.5);
                leftback.setPower(0.6);
                rightfr.setPower(0.5);
                rightback.setPower(-0.5);
            }
            while (runtime.seconds() < 1.4) {
                leftfr.setPower(0.5);
                leftback.setPower(0.6);
                rightfr.setPower(0.5);
                rightback.setPower(0.5);
            }
            while (runtime.seconds() < 2.7) {
                leftfr.setPower(-0.5);
                leftback.setPower(-0.6);
                rightfr.setPower(-0.5);
                rightback.setPower(-0.5);
            }
            while (runtime.seconds() < 3.5) {
                leftfr.setPower(0.0);
                leftback.setPower(0.0);
                rightfr.setPower(0.0);
                rightback.setPower(0.0);
                leftHook.setPosition(1);
                rightHook.setPosition(1);
            }
            while (runtime.seconds() < 4.2) { //bring foundation backwards
                leftfr.setPower(0.5);
                leftback.setPower(0.6);
                rightfr.setPower(0.5);
                rightback.setPower(0.5);
            }
            leftfr.setPower(0);
            leftback.setPower(0);
            rightfr.setPower(0);
            rightback.setPower(0);

            AcuTurn(-90, true);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }

    public void AcuTurn(double Degrees, boolean Clockwise) {
        Degrees = (Math.abs(Degrees));
        double moved = 0;

        double lastHead;


        telemetry.addData("Test Uday", true);
        telemetry.update();
        initHeading = angles.firstAngle;
        lastHead = angles.firstAngle;
        telemetry.addData("TestSTATEMENT:", moved != Degrees);
        telemetry.addData("Heading: ", angles.firstAngle);

        while (moved < (Degrees - 3)) {
            rightback.setDirection(DcMotor.Direction.REVERSE);
            rightfr.setDirection(DcMotor.Direction.REVERSE);
            leftfr.setDirection(DcMotor.Direction.REVERSE);
            leftback.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addData("Test Data", Degrees - Math.abs(angles.firstAngle - initHeading));
            telemetry.addData("Test Data 2", Math.abs(angles.firstAngle - initHeading));
            telemetry.addData("Moved: ", moved);
            telemetry.addData("Degrees: ", Degrees);
            telemetry.addData("Velocity", imu.getVelocity());


            if (Clockwise == true) {
                if ((Degrees - moved) > 70) {
                    leftfr.setPower(.65);
                    leftback.setPower(.65);
                    rightfr.setPower(.65);
                    rightback.setPower(.65);
                } else {
                    if ((Degrees - moved) > 40) {
                        leftfr.setPower(.35);
                        leftback.setPower(.35);
                        rightfr.setPower(.35);
                        rightback.setPower(.35);
                    } else {
                        if ((Degrees - moved) > 30) {
                            leftfr.setPower(.25);
                            leftback.setPower(.25);
                            rightfr.setPower(.25);
                            rightback.setPower(.25);
                        }

                        if ((Degrees - moved) > 10) {
                            leftfr.setPower(.2);
                            leftback.setPower(.2);
                            rightfr.setPower(.2);
                            rightback.setPower(.2);
                        } else {
                            if ((Degrees - moved) > 5) {
                                leftfr.setPower(.2);
                                leftback.setPower(.2);
                                rightfr.setPower(.2);
                                rightback.setPower(.2);
                            }
                        }
                    }
                }

            } else {
                leftfr.setPower(-.2);
                leftback.setPower(-.2);
                rightfr.setPower(-.2);
                rightback.setPower(-.2);
            }


            moved = moved + (Math.abs(angles.firstAngle - lastHead));
            lastHead = angles.firstAngle;
            telemetry.update();

        }
        sleep(500);
        telemetry.addData("Setting Power to 0", true);
        telemetry.update();
        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);


        return;
    }
    public  void composeTelemetry () {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });

    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees ( double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

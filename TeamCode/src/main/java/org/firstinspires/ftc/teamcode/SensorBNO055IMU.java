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
import java.lang.*;
import java.text.DecimalFormat;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import java.lang.Math;



import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "Sensor: BNO055 IMU", group = "Sensor")
// Comment this out to add to the opmode list
public class SensorBNO055IMU extends LinearOpMode
{




    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfr = null;
    private DcMotor leftback = null;
    private DcMotor rightfr = null;
    private DcMotor rightback = null;

    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;

    double firstHeading;

    double initHeading;
    double XY_Scale = 0;



    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    float currentAngle;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        leftfr = hardwareMap.get(DcMotor.class, "leftf");
        leftback = hardwareMap.get(DcMotor.class, "leftb");
        rightfr = hardwareMap.get(DcMotor.class, "rightf");
        rightback = hardwareMap.get(DcMotor.class, "rightb");




        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);



        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {

telemetry.update();
firstHeading = 30;
      AcuTurn2(90,true, true);
      stop();



//            tmove(1, runtime.seconds(), -.4,0,0);
//            tmove(2, runtime.seconds(), 0,0,0);
            //VIMove(0,100,.2);





            stop();



            /*
            while(Math.abs(angles.firstAngle - initHeading) > 90)
            {
                bMove(.2,0,0);
            }

*/



        }


    }
      public void straightMove ( double speed,
                               double time) // TI
    {
        telemetry.update();

        leftfr.setPower(speed);
        leftback.setPower(speed);
        rightfr.setPower(speed);
        rightback.setPower(speed);

        double initHeading = angles.firstAngle;



        runtime.reset();
        while(runtime.seconds() < time)
        {
            double leftPower = speed, rightPower = speed;
            telemetry.update();


            if (Math.abs(angles.firstAngle - initHeading) > 1)
            {

                double adjust = -Range.scale(Range.clip(Math.abs(angles.firstAngle - initHeading), 0, 8),0, 20, 0,1 );
                if((angles.firstAngle - initHeading) > 0)
                {
                    leftPower = leftPower + adjust;
                    rightPower = rightPower - adjust;
                }else
                {
                    leftPower = leftPower - adjust;
                    rightPower = rightPower + adjust;
                }
                telemetry.addData("Adjusting", true);
                telemetry.addData("Adjust:", adjust);
            }




            leftfr.setPower(Range.clip(leftPower, Constants.MIN_POWER, Constants.MAX_POWER));
            leftback.setPower(Range.clip(leftPower, Constants.MIN_POWER, Constants.MAX_POWER));
            rightfr.setPower(Range.clip(rightPower, Constants.MIN_POWER, Constants.MAX_POWER));
            rightback.setPower(Range.clip(rightPower, Constants.MIN_POWER, Constants.MAX_POWER));



        }
        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);

        telemetry.addData("Change:",angles.firstAngle- initHeading );
        telemetry.update();
        sleep(1000);

/*
        leftfr.setTargetPosition((int)(leftfr.getCurrentPosition() + Math.round(leftInches*Constants.COUNTS_PER_INCH)));
        leftback.setTargetPosition((int)(leftback.getCurrentPosition() + Math.round(leftInches*Constants.COUNTS_PER_INCH)));
        rightfr.setTargetPosition((int)(rightfr.getCurrentPosition() + Math.round(rightInches*Constants.COUNTS_PER_INCH)));
        rightback.setTargetPosition((int)(rightback.getCurrentPosition() + Math.round(rightInches*Constants.COUNTS_PER_INCH)));

        leftfr.setPower(speed);
        leftback.setPower(speed);
        rightfr.setPower(speed);
        rightback.setPower(speed);

        leftfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(leftfr.isBusy() || leftback.isBusy() || rightfr.isBusy() || rightback.isBusy())
        {
            telemetry.addData("Running:", true);

        }

        leftfr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);
*/

    }
    public void AcuTurn2 ( double Degrees, boolean Clockwise, boolean First) {
        Degrees = (Math.abs(Degrees));
        double moved = 0;


        double lastHead;


        telemetry.addData("Test Uday", true);
        telemetry.update();
        initHeading = angles.firstAngle;

        if(First == true)
        {
            moved =firstHeading - (angles.firstAngle) ;
        }
        lastHead = angles.firstAngle;
        telemetry.addData("TestSTATEMENT:", moved != Degrees);
        telemetry.addData("Heading: ", angles.firstAngle);

        while (moved < (Degrees)) {
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
                    leftfr.setPower(.45);
                    leftback.setPower(.45);
                    rightfr.setPower(.45);
                    rightback.setPower(.45);
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


    public void strafeMove ( double speed,
                             double time)
    {
        telemetry.update();

        leftfr.setPower(speed);
        leftback.setPower(speed);
        rightfr.setPower(speed);
        rightback.setPower(speed);

        double initHeading = angles.firstAngle;





        runtime.reset();
        while(runtime.seconds() < time)
        {
            double leftDiag = speed, rightDiag = speed;
            telemetry.update();


            if (Math.abs(angles.firstAngle - initHeading) > 1)
            {

                double adjust = -Range.scale(Range.clip(Math.abs(angles.firstAngle - initHeading), 0, 25),0, 25, 0,1 );
                if((angles.firstAngle - initHeading) > 0)
                {
                    leftbackPower = leftbackPower + adjust;
                    rightfrPower = rightfrPower + adjust;
                    leftfrPower = leftfrPower - adjust;
                    rightbackPower = leftfrPower - adjust;
                }else
                {
                    leftbackPower = leftbackPower - adjust;
                    rightfrPower = rightfrPower - adjust;
                    leftfrPower = leftfrPower + adjust;
                    rightbackPower = leftfrPower + adjust;
                }
                telemetry.addData("Adjusting", true);
                telemetry.addData("Adjust:", adjust);
            }


            telemetry.addData("Left FR", leftfr.getPower());
            telemetry.addData("Left BC", leftback.getPower());
            telemetry.addData("Right FR", rightfr.getPower());
            telemetry.addData("Right BK", rightback.getPower());

            leftfr.setPower(Range.clip(leftfrPower, Constants.MIN_POWER, Constants.MAX_POWER));
            leftback.setPower(Range.clip(-leftbackPower, Constants.MIN_POWER, Constants.MAX_POWER));
            rightfr.setPower(Range.clip(-rightfrPower, Constants.MIN_POWER, Constants.MAX_POWER));
            rightback.setPower(Range.clip(rightbackPower, Constants.MIN_POWER, Constants.MAX_POWER));



        }
        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);

        telemetry.addData("Change:",angles.firstAngle- initHeading );
        telemetry.update();
        sleep(1000);

/*
        leftfr.setTargetPosition((int)(leftfr.getCurrentPosition() + Math.round(leftInches*Constants.COUNTS_PER_INCH)));
        leftback.setTargetPosition((int)(leftback.getCurrentPosition() + Math.round(leftInches*Constants.COUNTS_PER_INCH)));
        rightfr.setTargetPosition((int)(rightfr.getCurrentPosition() + Math.round(rightInches*Constants.COUNTS_PER_INCH)));
        rightback.setTargetPosition((int)(rightback.getCurrentPosition() + Math.round(rightInches*Constants.COUNTS_PER_INCH)));

        leftfr.setPower(speed);
        leftback.setPower(speed);
        rightfr.setPower(speed);
        rightback.setPower(speed);

        leftfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(leftfr.isBusy() || leftback.isBusy() || rightfr.isBusy() || rightback.isBusy())
        {
            telemetry.addData("Running:", true);

        }

        leftfr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);
*/

    }



    public double getAcceleration(boolean x, boolean y)
    {
        int TimesRan = 0;



        double [] temp = new double[3];
        if(x == true)
        {
            while(TimesRan < 3)
            {
                temp[TimesRan] = gravity.xAccel * gravity.xAccel;
                telemetry.update();
            }
        }
        if(y == true)
        {
            while(TimesRan < 3)
            {
                temp[TimesRan] = gravity.yAccel * gravity.yAccel;
                telemetry.update();
            }
        }
        Arrays.sort(temp);

        return temp[1];
    }

    void VIMove(double Angle , double Distance, double Power)
    {
        telemetry.update();
        double Moved = 0;
        double xAcceleration;
        double yAcceleration;
        double initYAccelration = getAcceleration(false,true);
        double initXAccelration = getAcceleration(true,false);
        double xMoved;
        double yMoved;

        double xLastSpeed = 0;
        double yLastSpeed = 0;

        double xSpeed;
        double ySpeed;

        double lastTime = 0; //seconds
//        double lastXAcceleration;
//        double lastYAcceleration;


        while (Moved < Distance)
        {
            telemetry.update();

            xAcceleration = getAcceleration(true,false) - initXAccelration;
            yAcceleration = getAcceleration(false,true) - initYAccelration;

            xSpeed = xLastSpeed +  xAcceleration/(runtime.seconds() - lastTime);
            ySpeed = yLastSpeed +  yAcceleration/(runtime.seconds() - lastTime);

            xMoved = xSpeed *(runtime.seconds() - lastTime);
            yMoved = ySpeed *(runtime.seconds() - lastTime);

            Moved = Math.pow(Math.pow(xMoved, 2) + Math.pow(yMoved, 2), .5);





            telemetry.addData("Moved", Moved);
            telemetry.addData("XMoved:", xMoved);
            telemetry.addData("YMoved:", yMoved);
            lastTime = runtime.seconds();
        }
    }
    void vMove(double Angle, double Power)
    {
        double counter =0;


        double Vx = 0;//-(Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y)*180/Math.PI);
        double Vy = Math.sin(Angle);

        double Turn = Math.cos(Angle);

        leftfrPower = Vy + Vx + Constants.WHEEL_DIST_H*Turn + Constants.WHEEL_DIST_V*Turn;
        rightfrPower = Vy -Vx - Constants.WHEEL_DIST_H*Turn - Constants.WHEEL_DIST_V*Turn;
        leftbackPower = Vy + Vx -Constants.WHEEL_DIST_H*Turn - Constants.WHEEL_DIST_V*Turn;
        rightbackPower = Vy-Vx +Constants.WHEEL_DIST_H*Turn + Constants.WHEEL_DIST_V*Turn;








        telemetry.addData("Counter:", counter);

        double max1;
        double max2;
        double max;


        double min1;
        double min2;
        double min;

        double average;



        max1 = Math.max(Math.abs(leftfrPower), Math.abs(leftbackPower));
        max2 = Math.max(Math.abs(rightfrPower),Math.abs(rightbackPower));

        max = Math.max(max1, max2);







        leftfrPower = leftfrPower/(Math.abs(Power));
        leftbackPower = leftbackPower/(Math.abs(Power));
        rightfrPower = rightfrPower/(Math.abs(Power));
        rightbackPower = rightbackPower/(Math.abs(Power));



        leftfr.setPower(leftfrPower);
        leftback.setPower(leftbackPower);
        rightfr.setPower(rightfrPower);
        rightback.setPower(rightbackPower);


    }
    void AcuMove(double StraifMeters, double TankMeters)
    {


        telemetry.update();
        TankMeters = TankMeters*10000;




        double initXaccel = gravity.yAccel;
        double Xaccel;
        double Yaccel = 0;
        double Speed =  0;


        double lastTime = 0;
        double lastSpeed = 0;


        double movedTank = 0;

        double runTimes = 0;
        double initialTime =runtime.seconds();

        while((runtime.seconds() - initialTime)< 5)
        {
            idle();
        }




        while(movedTank < TankMeters){
            telemetry.update();
            runTimes = runTimes +1;

            Xaccel = (gravity.yAccel)-initXaccel;



            if(Xaccel < .05 && Xaccel > -.05)
            {
                Xaccel = 0;
                telemetry.addData("Zeroed:", "True");
            }
            else
            {
                telemetry.addData("Zeroed:", "False");
            }



            Speed = lastSpeed + (Xaccel/(runtime.seconds() - lastTime));
            telemetry.addData("Speed:", Speed + "mm/s");
            telemetry.addData("X acceleration:", Xaccel);
            telemetry.addData("Y acceleration:", Yaccel);



            telemetry.addData("Equation(Speed)=", lastSpeed+"+(" +Xaccel +"/(",runtime.seconds() + "-" +lastTime+"))" );















            telemetry.addData("Moved", movedTank/10000);

            telemetry.addData("Times Run:", runTimes);

            movedTank = movedTank + (Speed*(runtime.seconds() - lastTime));
            telemetry.addData("Moved", movedTank);









//            if ((movedTank/TankMeters) > .9){
//                leftfr.setPower(.3);
//                leftback.setPower(.3);
//                rightback.setPower(-.3);
//                rightfr.setPower(-.3);
//
//            }else{
//                if((TankMeters - movedTank) > 4)
//                {
//                    leftfr.setPower(.7);
//                    leftback.setPower(.7);
//                    rightback.setPower(-.7);
//                    rightfr.setPower(-.7);
//                }else{
//                    if((TankMeters - movedTank) > 3)
//                    {
//                        leftfr.setPower(.5);
//                        leftback.setPower(.5);
//                        rightback.setPower(-.5);
//                        rightfr.setPower(-.5);
//                    }else{
//                        if ((TankMeters - movedTank) > 1)
//                        {
//                            leftfr.setPower(.3);
//                            leftback.setPower(.3);
//                            rightback.setPower(-.3);
//                            rightfr.setPower(-.3);
//                        }else{
//
//                            leftfr.setPower(.2);
//                            leftback.setPower(.2);
//                            rightback.setPower(-.2);
//                            rightfr.setPower(-.2);
//
//                        }
//                    }
//                }
//            }




            lastTime = runtime.seconds();





        }

        telemetry.addData("Moved Post Finish", movedTank*1000);
        telemetry.update();
        leftfr.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);
        rightfr.setPower(0);


        initialTime = runtime.seconds();
        while((runtime.seconds() - initialTime)< 5)
        {
            idle();
        }
        return;









    }
    public static void roundAndPrint(double n, int round2DecimalPlace) {
        String temp;
        BigDecimal instance = new BigDecimal(Double.toString(n));
        instance = instance.setScale(round2DecimalPlace, RoundingMode.HALF_UP);

    }

    public void Degree90Turn(double Degrees) {
        Degrees = (Math.abs(Degrees));
        double moved = 0;

        double lastHead;
        initHeading = angles.firstAngle;
        lastHead = angles.firstAngle;


        telemetry.addData("Initheading:", initHeading);
        telemetry.addData("LastHead:", lastHead);
        telemetry.update();
        currentAngle = 0;

        while (currentAngle <= 90 && !isStopRequested()) {
            rightback.setDirection(DcMotor.Direction.REVERSE);
            rightfr.setDirection(DcMotor.Direction.REVERSE);
            leftfr.setDirection(DcMotor.Direction.REVERSE);
            leftback.setDirection(DcMotor.Direction.REVERSE);
            telemetry.update();
            currentAngle = Math.abs(angles.firstAngle);
            if (currentAngle < 50) {
                leftfr.setPower(.5);
                leftback.setPower(.5);
                rightfr.setPower(.5);
                rightback.setPower(.5);
            }
          else if (currentAngle < 70) {
                leftfr.setPower(.25);
                leftback.setPower(.25);
                rightfr.setPower(.25);
                rightback.setPower(.25);
            }
          else if(currentAngle<90)
            {
                leftfr.setPower(.2);
                leftback.setPower(.2);
                rightfr.setPower(.2);
                rightback.setPower(.2);

            }
          else if(currentAngle>90)
            {
                leftfr.setPower(-.15);
                leftback.setPower(-.15);
                rightfr.setPower(-.15);
                rightback.setPower(-.15);
            }
            telemetry.addData("Setting Power:", rightback.getPower());
            telemetry.update();
        }
            leftfr.setPower(0);
            leftback.setPower(0);
            rightfr.setPower(0);
            rightback.setPower(0);
            telemetry.addData(" Outside", true);
            telemetry.update();
            sleep(40000);

            leftfr.setDirection(DcMotor.Direction.FORWARD);
            leftback.setDirection(DcMotor.Direction.FORWARD);
            rightfr.setDirection(DcMotor.Direction.REVERSE);
            rightback.setDirection(DcMotor.Direction.REVERSE);



    }















    public void AcuTurn ( double Degrees, boolean Clockwise) {
        Degrees = (Math.abs(Degrees));
        double moved = 0;

        double lastHead;


        telemetry.addData("Test Uday", true);
        telemetry.update();
        initHeading = angles.firstAngle;
        lastHead = angles.firstAngle;

        telemetry.addData("TestSTATEMENT:", moved != Degrees);
        telemetry.addData("Heading: ", angles.firstAngle);

        while (moved <= (Degrees)) {
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
                    leftfr.setPower(.25);
                    leftback.setPower(.25);
                    rightfr.setPower(.25);
                    rightback.setPower(.25);
                } else {
                    if ((Degrees - moved) > 40) {
                        leftfr.setPower(.25);
                        leftback.setPower(.25);
                        rightfr.setPower(.25);
                        rightback.setPower(.25);
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
                            if ((Degrees - moved) > 2) {
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
            if(moved>90)
            {
                leftfr.setPower(-.2);
                leftback.setPower(-.2);
                rightfr.setPower(-.2);
                rightback.setPower(-.2);
                moved = moved - (Math.abs(angles.firstAngle - lastHead));

            }
        }
        sleep(500);
        telemetry.addData("Setting Power to 0", true);
        telemetry.update();
        leftfr.setPower(0);
        leftback.setPower(0);
        rightfr.setPower(0);
        rightback.setPower(0);
        telemetry.addData("Moved After Loop",moved);
        telemetry.update();
        sleep(60000);
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);


        return;

    }




    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }






















    public void tmove(double runSeconds, double presentRuntime, double turnPower, double straifPower, double tankPower)

    {

        while(runtime.seconds() < (runSeconds+presentRuntime))
        {
            bMove(turnPower, straifPower, tankPower);

        }
    }

    public void bMove(double turnPower, double straifPower, double tankPower )
    {
        double leftfrPower;
        double leftbackPower;
        double rightfrPower;
        double rightbackPower;


        turnPower = turnPower*-1;

        tankPower = tankPower*-1;

        if (turnPower < -0.1) {
            leftfrPower = Range.clip(-turnPower, -1.0, 1.0);
            leftbackPower = Range.clip(-turnPower, -1.0, 1.0);
            rightfrPower = Range.clip(turnPower, -1.0, 1.0);
            rightbackPower = Range.clip(turnPower, -1.0, 1.0);
        } else if (turnPower > 0.1) {
            leftfrPower = Range.clip(-turnPower, -1.0, 1.0);
            leftbackPower = Range.clip(-turnPower, -1.0, 1.0);
            rightfrPower = Range.clip(turnPower, -1.0, 1.0);
            rightbackPower = Range.clip(turnPower, -1.0, 1.0);
        } else if (straifPower < -0.1) {
            leftfrPower = Range.clip(-straifPower, -1.0, 1.0);
            leftbackPower = Range.clip(straifPower, -1.0, 1.0);
            rightfrPower = Range.clip(straifPower, -1.0, 1.0);
            rightbackPower = Range.clip(-straifPower, -1.0, 1.0);
        } else if (straifPower > 0.1) {
            leftfrPower = Range.clip(-straifPower, -1.0, 1.0);
            leftbackPower = Range.clip(straifPower, -1.0, 1.0);
            rightfrPower = Range.clip(straifPower, -1.0, 1.0);
            rightbackPower = Range.clip(-straifPower, -1.0, 1.0);
        } else if (tankPower < -0.1) {
            leftbackPower = Range.clip(tankPower, -1.0, 1.0);
            rightbackPower = Range.clip(tankPower, -1.0, 1.0);
            leftfrPower = Range.clip(tankPower, -1.0, 1.0);
            rightfrPower = Range.clip(tankPower, -1.0, 1.0);
        } else if (tankPower > 0.1) {
            leftfrPower = Range.clip(tankPower, -1.0, 1.0);
            rightfrPower = Range.clip(tankPower, -1.0, 1.0);
            leftbackPower = Range.clip(tankPower, -1.0, 1.0);
            rightbackPower = Range.clip(tankPower, -1.0, 1.0);
        } else {
            tankPower = 0.0;
            turnPower = 0.0;
            leftfrPower = Range.clip(tankPower, -1.0, 1.0);
            rightfrPower = Range.clip(tankPower, -1.0, 1.0);
            leftbackPower = Range.clip(tankPower, -1.0, 1.0);
            rightbackPower = Range.clip(tankPower, -1.0, 1.0);
        }
        leftfr.setPower(leftfrPower);
        leftback.setPower(leftbackPower);
        rightfr.setPower(rightfrPower);
        rightback.setPower(rightbackPower);


    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="MR RangeSensorTest", group="Linear Opmode")
public class DistanceSensorTest extends LinearOpMode {
    DcMotor leftfr, leftback, rightfr, rightback;
    Servo leftHook, rightHook;

ModernRoboticsI2cRangeSensor rangeMRSensor;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        leftfr = hardwareMap.get(DcMotor.class, "leftf");
        leftback = hardwareMap.get(DcMotor.class, "leftb");
        rightfr = hardwareMap.get(DcMotor.class, "rightf");
        rightback = hardwareMap.get(DcMotor.class, "rightb");

//        leftWheels = hardwareMap.get(CRServo.class, "lw");
//        rightWheels = hardwareMap.get(CRServo.class, "rw");
        //rackPinionUD = hardwareMap.get (CRServo.class, "rpUpDown");
        //rackPinionLR = hardwareMap.get (CRServo.class, "rpLeftRight");

        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");

   //     autoHook = hardwareMap.get(Servo.class, "autoHook");



        rangeMRSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeMRSensor");
      //  colorMRSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorMRSensor");




        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);



        //rackPinionUD.setDirection(CRServo.Direction.FORWARD);
        //rackPinionLR.setDirection(CRServo.Direction.FORWARD);

        //rightfr.setmode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftfr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //int MOTORTICKS = 1680;

        leftHook.setPosition(0);
        rightHook.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            while(rangeMRSensor.getDistance(DistanceUnit.INCH)<25)
            {
                telemetry.addData("MR Range Sensor",rangeMRSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                leftfr.setPower(0.2);
                leftback.setPower(-0.2);
                rightfr.setPower(-0.2);
                rightback.setPower(0.2);
            }
            while(rangeMRSensor.getDistance(DistanceUnit.INCH)>25) {
                telemetry.addData("MR Range Sensor",rangeMRSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                leftfr.setPower(-0.2);
                leftback.setPower(0.2);
                rightfr.setPower(0.2);
                rightback.setPower(-0.2);
            }
            double distance = rangeMRSensor.getDistance(DistanceUnit.INCH);
            leftfr.setPower(0);
            leftback.setPower(0);
            rightfr.setPower(0);
            rightback.setPower(0);
            telemetry.addData("Distance to Wall Variable",distance);
            telemetry.update();
            sleep(40000);
            stop();



            //        telemetry.addData("Connection: ", MRcs1.getConnectionInfo());

        }

    }
}










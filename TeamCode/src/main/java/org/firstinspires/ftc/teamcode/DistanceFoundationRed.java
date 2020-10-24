
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous(name="DistanceFoundationTest", group="Linear Opmode")
public class DistanceFoundationRed extends LinearOpMode {
    DcMotor rightfDrive, rightbDrive, leftfDrive, leftbDrive;
    Servo leftHook, rightHook;
    int MOTOR_TICKS = 1680;
    int TURN_TICKS = 420;


    private DistanceSensor sensorRange;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "distanceSensor1");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        waitForStart();
        while (opModeIsActive()) {
            // generic DistanceSensor methods.


            // Rev2mDistanceSensor specific methods.

            if (sensorRange.getDistance(DistanceUnit.INCH) <= 30)
            {
                telemetry.addData("Detected:", "True");
                telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
                telemetry.update();
            }

            else {
                telemetry.addData("Detected:", "False");
                telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
                telemetry.update();
            }


        }
    }
}

            /*
            if (distanceSensor.getDistance(DistanceUnit.INCH)<=48)
            {
                rightHook.setPosition(1);
                leftHook.setPosition(1);
            }
            */



         /*           sleep(2000);
                    rightbDrive.setTargetPosition(MOTOR_TICKS2);
                    rightbDrive.setPower(0.5);
                    rightfDrive.setPower(-0.5);
                    leftfDrive.setPower(0.5);
                    leftbDrive.setPower(-0.5);
                    rightbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    if (!rightbDrive.isBusy()) {
                        leftfDrive.setPower(0);
                        leftbDrive.setPower(0);
                        rightfDrive.setPower(0);
                        rightbDrive.setPower(0);
                        telemetry.addData("Motors:", "Finished");
                        telemetry.update();
                    }

                    sleep(2000);
                    telemetry.addData("Motors: ", "Target Position Reached");
                    telemetry.update();
                    stop();
                }

*/


















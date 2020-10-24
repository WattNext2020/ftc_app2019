
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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

@Autonomous(name="OpticalDS", group="Linear Opmode")
public class OpticalDS extends LinearOpMode {
    DcMotor leftfr, leftback, rightfr, rightback;
    Servo leftHook, rightHook;

    ModernRoboticsI2cColorSensor  MRcs1;
    DistanceSensor sensorDistance;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

sensorDistance=hardwareMap.get(DistanceSensor.class,"sensorDistance");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {


            telemetry.addData("Optical Distance Sensor:",sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.update();

            while(sensorDistance.getDistance(DistanceUnit.CM)<7)
            {
                telemetry.addData("SKYSTONE",sensorDistance.getDistance(DistanceUnit.CM));
                telemetry.update();

            }

            //        telemetry.addData("Connection: ", MRcs1.getConnectionInfo());

        }

    }
}











package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="ColorSensor", group="Linear Opmode")
@Disabled
public class ColorSensor extends LinearOpMode {
    DcMotor leftfr, leftback, rightfr, rightback;
    Servo leftHook, rightHook;

    ModernRoboticsI2cColorSensor  MRcs1;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        leftfr = hardwareMap.get(DcMotor.class, "leftf");
        leftback = hardwareMap.get(DcMotor.class, "leftb");
        rightfr = hardwareMap.get(DcMotor.class, "rightf");
        rightback = hardwareMap.get(DcMotor.class, "rightb");


        //rackPinionUD = hardwareMap.get (CRServo.class, "rpUpDown");
        //rackPinionLR = hardwareMap.get (CRServo.class, "rpLeftRight");


        MRcs1 = hardwareMap.get(ModernRoboticsI2cColorSensor.class,"MRcs1");
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);


        //rackPinionUD.setDirection(CRServo.Direction.FORWARD);
        //rackPinionLR.setDirection(CRServo.Direction.FORWARD);

        rightHook.setDirection(Servo.Direction.REVERSE);
        //rightfr.setmode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftfr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //int MOTORTICKS = 1680;

        leftHook.setPosition(0);
        rightHook.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            while(!(MRcs1.red()>0))
            {
                leftfr.setPower(0.5);
                leftback.setPower(-0.5);
                rightfr.setPower(-0.5);
                rightback.setPower(0.5);
            }
            leftfr.setPower(0);
            leftback.setPower(0);
            rightfr.setPower(0);
            rightback.setPower(0);
            stop();


                   telemetry.addData("Red:",MRcs1.red());
                  telemetry.addData("Green:",MRcs1.green());
                  telemetry.addData("Blue:",MRcs1.blue());
                  telemetry.update();

            //        telemetry.addData("Connection: ", MRcs1.getConnectionInfo());

        }

    }
}










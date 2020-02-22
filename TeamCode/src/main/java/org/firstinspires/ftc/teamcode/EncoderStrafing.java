
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="StrafingTime", group="Linear Opmode")
public class EncoderStrafing extends LinearOpMode {
    DcMotor rightfr, rightback, leftfr, leftback;
    Servo leftHook, rightHook;
    int STRAFE_TICKS = 400;
    int MOTOR_TICKS2 = 1680;


    @Override
    public void runOpMode() throws InterruptedException {


        leftfr = hardwareMap.get(DcMotor.class, "leftf");
        leftback = hardwareMap.get(DcMotor.class, "leftb");
        rightfr = hardwareMap.get(DcMotor.class, "rightf");
        rightback = hardwareMap.get(DcMotor.class, "rightb");


        //rackPinionUD = hardwareMap.get (CRServo.class, "rpUpDown");
        //rackPinionLR = hardwareMap.get (CRServo.class, "rpLeftRight");

        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfr.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfr.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
double initTime=0;

        //rackPinionUD.setDirection(CRServo.Direction.FORWARD);
        //rackPinionLR.setDirection(CRServo.Direction.FORWARD);

        rightHook.setDirection(Servo.Direction.REVERSE);
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
runtime.startTime();

            //       telemetry.addData("MRcs1 name:",MRcs1.getI2cAddress());
            //        telemetry.addData("Connection: ", MRcs1.getConnectionInfo());

/*            telemetry.addLine(" Value:");
            telemetry.addData("Red Value",REVcs1.red());
            telemetry.addData("Green Value",REVcs1.green());
            telemetry.addData("Blue Value",REVcs1.blue());
            telemetry.update();

if (REVcs1.red()>450 && REVcs1.green()>300)
{
telemetry.addData("Yellow Stone Detected:","True");


}
else
{

    telemetry.addData("ellow Stone Detected:","False");

*/
            initTime=runtime.seconds();

            while((runtime.seconds()-initTime)<2.0)
            {
                rightback.setPower(-0.3);
                rightfr.setPower(0.3);
                leftfr.setPower(-0.3);
                leftback.setPower(0.3);
                telemetry.addData("Strafing"," ");
                telemetry.update();
            }
            rightback.setPower(0);
            rightfr.setPower(0);
            leftfr.setPower(0);
            leftback.setPower(0);
            stop();
            }
       /*         rightbDrive.setTargetPosition(rightbDrive.getCurrentPosition()+MOTOR_TICKS2);
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

        }

    }











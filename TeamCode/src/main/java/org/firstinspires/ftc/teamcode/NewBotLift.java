
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


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

@TeleOp(name="NewBotLift", group="Linear Opmode")
public class NewBotLift extends LinearOpMode {

public CRServo intakeLift;

        int MOTOR_TICKS = 820;
    int MOTOR_TICKS2 = 1680;

double intakeLiftPower;
    @Override
    public void runOpMode() throws InterruptedException {


       intakeLift = hardwareMap.get(CRServo.class,"rpUpDown");


        waitForStart();

        while (opModeIsActive()) {

          intakeLiftPower =gamepad1.right_stick_y;
          intakeLift.setPower(intakeLiftPower);

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

}
*/


        }
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













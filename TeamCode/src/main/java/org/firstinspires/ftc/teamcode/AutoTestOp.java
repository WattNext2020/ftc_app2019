
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.String;


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

@Autonomous(name="AutoTestOp", group="Linear Opmode")
public class AutoTestOp extends LinearOpMode {
    DcMotor rightfDrive, rightbDrive, leftfDrive, leftbDrive;


    @Override
    public void runOpMode() throws InterruptedException {


        rightfDrive = hardwareMap.get(DcMotor.class, "rightf");
        rightbDrive = hardwareMap.get(DcMotor.class, "rightb");
        leftfDrive  = hardwareMap.get(DcMotor.class, "leftf");
        leftbDrive = hardwareMap.get(DcMotor.class, "leftb");

        leftfDrive.setDirection(DcMotor.Direction.FORWARD);
        leftbDrive.setDirection(DcMotor.Direction.FORWARD);
        rightfDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbDrive.setDirection(DcMotor.Direction.REVERSE);

            waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.update();
            if (getRuntime()< 2.00) {
                rightbDrive.setPower(0.25);
                rightfDrive.setPower(0.25);
                leftfDrive.setPower(0.25);
                leftbDrive.setPower(0.25);
            }
             sleep(4000);
            if (getRuntime() >= 2.00 && getRuntime()<=4.00) {
                rightbDrive.setPower(-0.25);
                rightfDrive.setPower(-0.25);
                leftfDrive.setPower(-0.25);
                leftbDrive.setPower(-0.25);
            }
          //  sleep(4000);

        }

        // Tell the driver that initialization is complete.

    }
    }


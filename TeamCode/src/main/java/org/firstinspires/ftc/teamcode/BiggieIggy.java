package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="BiggieIggy", group="Linear Opmode")
@Disabled 
public class BiggieIggy extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Frontleft = null;
    private DcMotor Frontright = null;
    private DcMotor Backleft = null;
    private DcMotor Backright = null;


    {
        DcMotor arm;
        arm = null;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Frontleft  = hardwareMap.get(DcMotor.class, "Frontleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Backleft = hardwareMap.get(DcMotor.class,  "Backleft");
        Backright = hardwareMap.get(DcMotor.class, "Backright");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        Frontleft.setDirection(DcMotor.Direction.FORWARD);
        Frontright.setDirection(DcMotor.Direction.REVERSE);
        Backleft.setDirection(DcMotor.Direction.FORWARD);
        Backright.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double FrontleftPower;
            double FrontrightPower;
            double BackleftPower;
            double BackrightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            FrontleftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            FrontrightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            BackleftPower     = Range.clip(drive + turn, 1.0, -1.0);
            BackrightPower    = Range.clip(drive - turn, 1.0, -1.0);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            Frontleft.setPower(FrontleftPower);
            Frontright.setPower(FrontrightPower);
            Backleft.setPower(BackleftPower);
            Backright.setPower(BackrightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Frontleft (%.2f), Frontright (%.2f), Backleft (%.2f), Backright (%.2f) ", FrontleftPower, FrontrightPower, BackleftPower, BackrightPower);
            telemetry.update();
        }
    }
}


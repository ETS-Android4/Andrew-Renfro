package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Driver code", group="Linear Opmode")
public class DriverControl extends OpMode {

    MechanumHardware hw;

    @Override
    public void init() {
        hw = new MechanumHardware(hardwareMap);
        hw.init();
    }

    @Override
    public void loop() {
        hw.brake();

        if (Math.abs(gamepad1.left_stick_y) >= Math.abs(gamepad1.left_stick_x)) {
            hw.setDrivePower(gamepad1.left_stick_y);
        }

        else if (gamepad1.left_stick_x != 0) {
            hw.setTurnPower(gamepad1.left_stick_x);
        }

        if (gamepad1.right_stick_x != 0) {
            hw.setStrafePower(gamepad1.right_stick_x);
        }

        if (gamepad1.left_bumper) hw.moveArm1(1);
        else if (gamepad1.right_bumper) hw.moveArm1(-1);

        if (gamepad1.left_trigger != 0) hw.moveArm2(-1);
        else if (gamepad1.right_trigger != 0) hw.moveArm2(1);
    }
}

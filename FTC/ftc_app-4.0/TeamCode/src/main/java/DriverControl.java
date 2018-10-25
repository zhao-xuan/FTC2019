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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


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

@TeleOp(name="Driver Control", group="Linear Opmode")
//@Disabled
public class DriverControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrivef;
    private DcMotor rightDrivef;
    private DcMotor leftDriveb;
    private DcMotor rightDriveb;
    /*private DcMotor forklift;
    private CRServo handLeft;
    private CRServo handRight;
    private Servo sideArm;*/

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrivef  = hardwareMap.get(DcMotor.class, "left_drivef");
        rightDrivef = hardwareMap.get(DcMotor.class, "right_drivef");
        leftDriveb  = hardwareMap.get(DcMotor.class, "left_driveb");
        rightDriveb = hardwareMap.get(DcMotor.class, "right_driveb");
        /*forklift = hardwareMap.get(DcMotor.class, "forklift");
        handLeft = hardwareMap.get(CRServo.class, "handLeft");
        handRight = hardwareMap.get(CRServo.class, "handRight");
        sideArm = hardwareMap.get(Servo.class, "sideArm");*/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrivef.setDirection(DcMotor.Direction.REVERSE);
        rightDrivef.setDirection(DcMotor.Direction.FORWARD);
        leftDriveb.setDirection(DcMotor.Direction.REVERSE);
        rightDriveb.setDirection(DcMotor.Direction.FORWARD);
        /*forklift.setDirection(DcMotor.Direction.FORWARD);*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {



            //sideArm.setPosition(1);



            if (gamepad1.dpad_up){
                driveStraight(0.4);
            } else if (gamepad1.dpad_down) {
                driveStraight(-0.4);
            } else if (gamepad1.dpad_left) {
                //driveHorizontal(1);
                //lb,rf slow
                leftDrivef.setPower(-1);
                rightDrivef.setPower(1); //0.7
                leftDriveb.setPower(1); //0.7
                rightDriveb.setPower(-1);
            } else if (gamepad1.dpad_right) {
                //driveHorizontal(-1);
                //rf,lb slow
                leftDrivef.setPower(1);
                rightDrivef.setPower(-1);//0.7
                leftDriveb.setPower(-1);//0.7
                rightDriveb.setPower(1);
            }
            //turning
            else if (gamepad1.x) {
                turn(0.37);
            } else if (gamepad1.b) {
                turn(-0.37);
            } else {
                stopDriving();
            }

            //Control the forklift
            if (gamepad2.dpad_up){
                //forklift.setPower(-0.6);
            }
            else if (gamepad2.dpad_down){
                //forklift.setPower(0.6);
            }
            else{
                //forklift.setPower(0);
            }

            //Control the hand (servo)
            if(gamepad2.y){
                //close
                //handLeft.setPower(0.6);
            }
            else if (gamepad2.x){
                //open
               // handLeft.setPower(-0.6);
            }
            else{
                //handLeft.setPower(0);
            }

            if(gamepad2.b){
                //handRight.setPower(-0.6);
            }
            else if (gamepad2.a){
                //handRight.setPower(0.6);
            }
            else{
                //handRight.setPower(0);
            }

        }
    }
    public void driveStraight (double power){
        leftDrivef.setPower(power);
        rightDrivef.setPower(power);
        leftDriveb.setPower(power);
        rightDriveb.setPower(power);
    }
    public void stopDriving (){
        driveStraight(0);
    }

    public void turn (double power) {
        //1 turn left, -1 turn right
        leftDrivef.setPower(-power);
        rightDrivef.setPower(power);
        leftDriveb.setPower(-power);
        rightDriveb.setPower(power);
    }
    public void driveHorizontal  (double power){
        //1 to left, -1 to right
        leftDrivef.setPower(-power);
        rightDrivef.setPower(power);
        leftDriveb.setPower(power);
        rightDriveb.setPower(-power);
    }

}


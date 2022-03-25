////package org.firstinspires.ftc.teamcode.ftc16072;
////
////import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorSimple;
////
////@Autonomous(name = "Autonom", group = "a")
////public class Autonom extends LinearOpMode {
////    private String TFOD_MODEL_ASSET = "Skystone.tfite";
////    private String LABEL_FIRST_ELEMENT = "Stone";
////    private String LABEL_SECOND_ELEMENT = "SkyStone";
////
////    DcMotor leftFrontMotor = null;
////    DcMotor leftRearMotor = null;
////    DcMotor rightRearMotor = null;
////    DcMotor rightFrontMotor = null;
////
////    double diametru = 64;
////    double raza = diametru / 2;
////
////    @Override
////    public void runOpMode() throws InterruptedException {
////        initializare();
////
////        waitForStart();
////        if (opModeIsActive()) {
//////            rotire(90);
////
////            nush(2000, 200, 1,1,1,1);
//////            deplasere_encoder(2000, 2000, 2000, 2000);
////
//////            mergi_in_fata(1000);
////
////        }
////    }
////
////    public void mergi_in_fata(int milisecunde) {
////        deplasare(1, 1, 1, 1);
////        sleep(milisecunde);
////    }
////
////    public void rotire_grade(int grade) {
////
////    }
////
////    public void mergi_la_stanga() {
////
////    }
////
////    public void initializare() {
////        leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
////        rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
////        leftRearMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
////        rightRearMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
////
////        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
////        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
////        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
////        rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
////    }
////
////    public void nush(int distanta, int milisecunde, double lm1, double lm2, double rm1, double rm2) {
////        oprire_encoder();
////        run_without_encoders();
////        deplasare(lm1, lm2, rm1, rm2);
//////        sleep(milisecunde);
////
////        double circumferenta = 3.14 * 3.937;
////        double rotationNeeded = distanta / circumferenta;
////
////        int encoderDrivingTarget = (int) (rotationNeeded * 1120);
////
////        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////
////
////        leftFrontMotor.setTargetPosition(encoderDrivingTarget);
////        leftRearMotor.setTargetPosition(encoderDrivingTarget);
////        rightFrontMotor.setTargetPosition(encoderDrivingTarget);
////        rightRearMotor.setTargetPosition(encoderDrivingTarget);
////
////    }
////
////    public void deplasare(double lm1, double lm2, double rm1, double rm2) {
////        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////
////        leftFrontMotor.setPower(lm1);
////        rightFrontMotor.setPower(rm1);
////        leftRearMotor.setPower(lm2);
////        rightRearMotor.setPower(rm2);
////
////
////    }
////
////    public void run_without_encoders() {
////        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////    }
////
////    public void oprire_encoder() {
////        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////    }
////
////    public void deplasere_encoder(int lfm, int lrm, int rfm, int rrm) {
////        oprire_encoder();
////
//////        run_without_encoders();
////
////        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////        lfm /= 2.54;
////        rfm /= 2.54;
////        lrm /= 2.54;
////        rrm /= 2.54;
////
////        leftFrontMotor.setTargetPosition(lfm);
////        leftRearMotor.setTargetPosition(lrm);
////        rightFrontMotor.setTargetPosition(rfm);
////        rightRearMotor.setTargetPosition(rrm);
////
////
////    }
////
////    public void rotire(double alfa) {
////        oprire_encoder();
////
////        double distanta = (2 * 3.14 * raza * (alfa / 360.)) / 2.54;
////
////        double circumferenta = 3.14 * 3.937;
////        double rotationNeeded = distanta / circumferenta;
////
////        int encoderDrivingTarget = (int) (rotationNeeded * 1120);
////
////        deplasere_encoder(encoderDrivingTarget, encoderDrivingTarget, encoderDrivingTarget, encoderDrivingTarget);
////
////        leftFrontMotor.setPower(1);
////        leftRearMotor.setPower(1);
////        rightFrontMotor.setPower(1);
////        rightRearMotor.setPower(1);
////
////        while (leftFrontMotor.isBusy() || leftRearMotor.isBusy() || rightFrontMotor.isBusy() || rightRearMotor.isBusy()) {
////            telemetry.addData("pla:   ", encoderDrivingTarget);
////            telemetry.addData("Parcurcand distanta", distanta);
////            telemetry.addData("[*]Distanta Calcul", "Running to %7d :%7d", encoderDrivingTarget, encoderDrivingTarget);
////            telemetry.addData("[**]Distanta spate", "Running at %7d :%7d", leftRearMotor.getCurrentPosition(), rightRearMotor.getCurrentPosition());
////            telemetry.addData("[***]Distanta fata", "Running at %7d :%7d", leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition());
////
////            telemetry.update();
////
////        }
////    }
////
////}
////
////
////
////
//
//
//package org.firstinspires.ftc.teamcode.Matei.plm_nush;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Autonomous(name = "Autonom", group = "a")
//public class Autonom extends LinearOpMode {
//    DcMotor leftMotor;
//    DcMotor rightMotor;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        leftMotor = hardwareMap.dcMotor.get("left_motor");
//        rightMotor = hardwareMap.dcMotor.get("right_motor");
//
//        // You will need to set this based on your robot's
//        // gearing to get forward control input to result in
//        // forward motion.
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        // reset encoder counts kept by motors.
////        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // set motors to run forward for 5000 encoder counts.
////        leftMotor.setTargetPosition(2000);
////        rightMotor.setTargetPosition(2000);
//
////        leftMotor.setTargetPosition(1000);
////        rightMotor.setTargetPosition(1000);
//        // set motors to run to target encoder position and stop with brakes on.
////        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
//        waitForStart();
////
////
////        leftMotor.setPower(1);
////        rightMotor.setPower(1);
////
////        leftMotor.setTargetPosition(0);
////        rightMotor.setTargetPosition(0);
////
////
////        leftMotor.setTargetPosition(3000);
////        rightMotor.setTargetPosition(3000);
////
////        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////        leftMotor.setPower(-1);
////        rightMotor.setPower(-1);
//
//        DriveForwardDistance(1,3000);
//        DriveForwardDistance(1,-3000);
//
////        while (leftMotor.isBusy()) {
////
////        }
//
////        leftMotor.setPower(0.0);
////        rightMotor.setPower(0.0);
////
////        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////
////        leftMotor.setTargetPosition(0);
////        rightMotor.setTargetPosition(0);
////
////        leftMotor.setPower(0.0);
////        rightMotor.setPower(0.0);
//
//    }
//
//    public void DriveForwardDistance(double power, int distance) {
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        rightMotor.setTargetPosition(distance);
//        leftMotor.setTargetPosition(distance);
//
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        drivePower(power);
//
//        while (rightMotor.isBusy() && leftMotor.isBusy()) {
//
//        }
//
//        stopPower();
//
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }
//
//    public void drivePower(double power) {
//        leftMotor.setPower(power);
//        rightMotor.setPower(power);
//    }
//
//    public void stopPower() {
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//
//    }
//}
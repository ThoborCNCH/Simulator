// package org.firstinspires.ftc.teamcode.Matei.plm_nush;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//@Autonomous(name = "Satana", group = "a")
//public class Satana extends LinearOpMode {
//    //se pune cu rotile pe linia albastra din stanga
//    DcMotor lf, lr, rf, rr;
//    int ticks = 1440;
//    double diameter = 4.125;
//    double circuferinta = Math.PI * diameter;
//    double cpi = ticks / circuferinta;
//    double bias = 0.8;
//    int conversion = (int) (cpi * bias);
//    BNO055IMU imu;
//    Orientation lastAngles = new Orientation();
//    double globalAngle, correction;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initializare();
//        waitForStart();
//        if (opModeIsActive()) {
//            mergi(1, 28);
////            rotire(90, 0.3);
////          aici ar fi detectia, dar na, plm, csf, n-ai csf
//
//            int caz = (int) (Math.random() * 3 + 1);
//
//            if (caz == 1) {
//                mergiLaPunctulA();
//            } else if (caz == 2) {
//                mergiLaPunctulB();
//            } else {
//                mergiLaPunctulC();
//            }
//
//            setMotorsPower(0, 0, 0, 0);
//
//            while (opModeIsActive()) {
//                correction = checkDirection();
//                telemetry.update();
//            }
//        }
//    }
//
//    //trasee
//    private void mergiLaPunctulA() {
//        System.out.println("a");
//        mergi(1, 44);
//        strafe(1, 20);
//        telemetry.addData("no code: ", "aici ar lasa kkt ala jos");
//        telemetry.update();
//        strafe(1, -20);
//    }
//
//    private void mergiLaPunctulB() {
//        System.out.println("b");
//        mergi(1, 65);
//        telemetry.addData("no code: ", "aici ar lasa kkt ala jos");
//        telemetry.update();
//        mergi(1, -15);
//    }
//
//    private void mergiLaPunctulC() {
//        System.out.println("c");
//        mergi(1, 90);
//        strafe(1, 20);
//        telemetry.addData("no code: ", "aici ar lasa kkt ala jos");
//        telemetry.update();
//        strafe(1, -20);
//        mergi(1, -40);
//    }
//
//
//    private void initializare() {
//        lf = hardwareMap.dcMotor.get("front_left_motor");
//        lr = hardwareMap.dcMotor.get("back_left_motor");
//        rf = hardwareMap.dcMotor.get("front_right_motor");
//        rr = hardwareMap.dcMotor.get("back_right_motor");
//        imu = hardwareMap.get(BNO055IMU.class,"imu");
//
//        lf.setDirection(DcMotorSimple.Direction.REVERSE);
//        lr.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
//
//        imu.initialize(parameters);
//    }
//
//    //nu e folosit dar sa fie acolo in caz de ceva
//
//    //GYRO
//    private double getAngle() {
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double delta = angles.firstAngle - lastAngles.firstAngle;
//
//        if (delta < -180) {
//            delta += 360;
//        } else if (delta > 180) {
//            delta -= 360;
//        }
//
//        globalAngle += delta;
//        lastAngles = angles;
//
//        return globalAngle;
//    }
//
//    private double checkDirection() {
//        double correction, angle = getAngle(), gain = 0.1;
//        if (angle == 0) {
//            correction = 0;
//        } else {
//            correction = -angle;
//        }
//
//        correction *= gain;
//
//        return correction;
//    }
//
//    public void resetAngle() {
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        globalAngle = 0;
//    }
//
//    //rotire
//    public void rotire(int grade, double power) {
//        double lfm, lrm, rfm, rrm;
//
//        resetAngle();
//
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        if (grade < 0) {
//            lfm = power;
//            lrm = power;
//            rfm = -power;
//            rrm = -power;
//        } else if (grade > 0) {
//            lfm = -power;
//            lrm = -power;
//            rfm = power;
//            rrm = power;
//        } else return;
//
//
//        setMotorsPower(lfm, lrm, rfm, rrm);
//
//        if (grade < 0) {
//            while (opModeIsActive() && getAngle() == 0) {
//            }
//            while (opModeIsActive() && getAngle() > grade) {
//            }
//        } else {
//            while (opModeIsActive() && getAngle() < grade) {
//            }
//        }
//
//        setMotorsPower(0, 0, 0, 0);
//        sleep(1000);
//        resetAngle();
//    }
//
//
//    //ENCODERE
//    private void resetEncoders() {
//        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    private void setTargetEncoders(int lfm, int lrm, int rfm, int rrm) {
//        lf.setTargetPosition(lf.getCurrentPosition() + lfm);
//        lr.setTargetPosition(lr.getCurrentPosition() + lrm);
//        rf.setTargetPosition(rf.getCurrentPosition() + rfm);
//        rr.setTargetPosition(rr.getCurrentPosition() + rrm);
//
//        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    private void runUsingEncoders() {
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void setMotorsPower(double lfm, double lrm, double rfm, double rrm) {
//        lf.setPower(lfm);
//        lr.setPower(lrm);
//        rf.setPower(rfm);
//        rr.setPower(rrm);
//    }
//
//    private void mergi(double power, int distance) {
//        int move = distance * conversion;
//        resetEncoders();
//        runUsingEncoders();
//        setTargetEncoders(move, move, move, move);
//        setMotorsPower(power, power, power, power);
//        while (lf.isBusy() && lr.isBusy() && rf.isBusy() && rr.isBusy()) {
//        }
//        setMotorsPower(0, 0, 0, 0);
//    }
//
//    private void strafe(double power, int distance) {
//        int move = distance * conversion;
//        resetEncoders();
//        runUsingEncoders();
//        setTargetEncoders(-move, move, move, -move);
//        setMotorsPower(power, power, power, power);
//        while (lf.isBusy() && lr.isBusy() && rf.isBusy() && rr.isBusy()) {
//        }
//        setMotorsPower(0, 0, 0, 0);
//    }
//
//}
////
/////*
////
////               ######                ######
////             ###     ####        ####     ###
////            ##          ###    ###          ##
////            ##             ####             ##
////            ##             ####             ##
////            ##           ##    ##           ##
////            ##         ###      ###         ##
////             ##  ########################  ##
////          ######    ###            ###    ######
////      ###     ##    ##              ##    ##     ###
////   ###         ## ###      ####      ### ##         ###
////  ##           ####      ########      ####           ##
//// ##             ###     ##########     ###             ##
////  ##           ####      ########      ####           ##
////   ###         ## ###      ####      ### ##         ###
////      ###     ##    ##              ##    ##     ###
////          ######    ###            ###    ######
////             ##  ########################  ##
////            ##         ###      ###         ##
////            ##           ##    ##           ##
////            ##             ####             ##
////            ##             ####             ##
////            ##          ###    ###          ##
////             ###     ####        ####     ###
////               ######                ######
////
////
////     */
//
////
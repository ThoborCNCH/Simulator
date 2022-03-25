package org.firstinspires.ftc.teamcode.Matei.altu;

import com.qualcomm.hardware.bosch.BNO055IMU;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class MickeyMouse extends LinearOpMode {
    public DcMotor lf = null, rf = null, lr = null, rr = null;
    HardwareMap Hmap = null;

    //        pt encodere
    // public static final double ticks = 145.6;
    public static final double ticks = 1150;
    public static final double diameter = 4.125;
    public static final double circuferenta = Math.PI * diameter;
    public static final double cpi = ticks / circuferenta;
    public static final double bias = 0.8;
    public static final int conversion = (int) (cpi * bias);

    //        pt gyro
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double global = 0;

    private static final String KEY =
            "ARhzqPT/////AAABmcks6V9uRE/4vJE+8qBUvnsYPXUfYlpJ8y+pzVN/GpzCrJsVanetvGKZxaJMs+3LmpTosqzKWHhdAiOzqd3kFmr4WYOWRErWkQuuVRx5/merGbBTYOAKQ9rkri+O3XR/l3bWk3zVlXUH7wXisifJcM2xoXGON4lYuETqenXu4NFfqOXkDGWI1nBNMM1dFW6AhLEuGt0R1TP6ToWiA1rk6dBvg7W3jGDi7eGYdvQhuo5I+6/ffn/OAyWnt+5DiJFVK365Cubaa0IE5xO3J4SSNcVXaho39lO5o7EhtCmqO2icWi8bYv7o+DHXWPsKfPByyrKSjEaXpvBNQ6S7P5pw9p5I5t6XafbS2LxYE5AJ6zH6";

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

//    public  VuforiaLocalizer vuforia;
//
//    public TFObjectDetector tfod;
//
//    public int este = 0;
//
//    public String detect(){
//        String ce = "";
//        if (opModeIsActive() && este == 0) {
//            while (opModeIsActive() && este == 0) {
//                if (tfod != null) {
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//                        if (updatedRecognitions.size() == 0 ) {
//                            telemetry.addData("TFOD", "No items detected.");
//                            telemetry.addData("Target Zone", "A");
//                            ce = "A";
//                        } else {
//                            // list is not empty.
//                            // step through the list of recognitions and display boundary info.
//                            int i = 0;
//                            for (Recognition recognition : updatedRecognitions) {
//                                if (recognition.getLabel().equals("Single")) {
//                                    ce = "B";
//                                } else if (recognition.getLabel().equals("Quad")) {
//                                    ce = "C";
//                                } else {
//                                    ce = "NU";
//                                }
//                            }
//                        }
//                        telemetry.update();
//                    }
//                }
//            }
//        }
//
//        return ce;
//    }
//
//    public void initVuforia(){
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = KEY;
////        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
////        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//    }
//
  /*  public void initTfod(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorView", "id",  hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        tfodParameters.minResultConfidence = 0.8f;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
*/
    public MickeyMouse() {}

    public void init(HardwareMap hMap) {
        Hmap = hMap;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        lf = hardwareMap.dcMotor.get("front_left_motor");
        rf = hardwareMap.dcMotor.get("front_right_motor");
        lr = hardwareMap.dcMotor.get("back_left_motor");
        rr = hardwareMap.dcMotor.get("back_right_motor");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = false;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
//        initVuforia();
//        initTfod();
//
//        if(tfod != null){
//            tfod.activate();
//            tfod.setZoom(2.5, 16.0/9.0);
//        }
    }

    public void mergiFata(double power, int distance) {
        Orientation angles;
        double lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        double error;
        double bias = 3 / 360.0;
        int lft = lf.getCurrentPosition() + (distance * conversion);
        int rft = rf.getCurrentPosition() + (distance * conversion);
        int lrt = lr.getCurrentPosition() + (distance * conversion);
        int rrt = rr.getCurrentPosition() + (distance * conversion);

        while (opModeIsActive() &&
                (power > 0 && lf.getCurrentPosition() < lft && rf.getCurrentPosition() < rft && lr.getCurrentPosition() < lrt && rr.getCurrentPosition() < rrt) ||
                (power < 0 && lf.getCurrentPosition() > lft && rf.getCurrentPosition() > rft && lr.getCurrentPosition() > lrt && rr.getCurrentPosition() > rrt)
        ) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            double angle = lastAngle - angles.firstAngle;
            error = (lastAngle - angle) * bias;
            setMotorsPower((power - error), (power - error), (power + error), (power + error));
            // setMotorsPower(power, power, power, power);
            telemetry.addData("lf", lf.getCurrentPosition());
            telemetry.addData("rf", rf.getCurrentPosition());
            telemetry.addData("lr", lr.getCurrentPosition());
            telemetry.addData("rr", rr.getCurrentPosition());
            telemetry.update();

        }
        setMotorsPower(0, 0, 0, 0);
    }

    private void mergiSpate(double power, int distance) {
        mergiFata(-power, -distance);
    }

    private void strafeRight(double power, int distance) {
        Orientation angles;
        double lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        double error;
        double bias = 3 / 360.0;
        double lft = lf.getCurrentPosition() + (distance * conversion);
        double rft = rf.getCurrentPosition() - (distance * conversion);
        double lrt = lr.getCurrentPosition() - (distance * conversion);
        double rrt = rr.getCurrentPosition() + (distance * conversion);

        while (opModeIsActive() && lf.getCurrentPosition() < lft && rf.getCurrentPosition() > rft && lr.getCurrentPosition() > lrt && rr.getCurrentPosition() < rrt) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            double angle = lastAngle - angles.firstAngle;
            error = (lastAngle - angle) * bias;
            setMotorsPower((power - error), -(power - error), -(power + error), (power + error));
            telemetry.addData("lf", lf.getCurrentPosition());
            telemetry.addData("rf", rf.getCurrentPosition());
            telemetry.addData("lr", lr.getCurrentPosition());
            telemetry.addData("rr", rr.getCurrentPosition());
            telemetry.update();

        }
        setMotorsPower(0, 0, 0, 0);
    }

    private void strafeLeft(double power, int distance) {
        Orientation angles;
        double lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        double error;
        double bias = 3 / 360.0;
        double lft = lf.getCurrentPosition() - (distance * conversion);
        double rft = rf.getCurrentPosition() + (distance * conversion);
        double lrt = lr.getCurrentPosition() + (distance * conversion);
        double rrt = rr.getCurrentPosition() - (distance * conversion);

        while (opModeIsActive() && lf.getCurrentPosition() > lft && rf.getCurrentPosition() < rft && lr.getCurrentPosition() < lrt && rr.getCurrentPosition() > rrt) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            double angle = lastAngle - angles.firstAngle;
            error = (lastAngle - angle) * bias;
            setMotorsPower(-(power + error), (power + error), (power - error), -(power - error));
            telemetry.addData("lf", lf.getCurrentPosition());
            telemetry.addData("rf", rf.getCurrentPosition());
            telemetry.addData("lr", lr.getCurrentPosition());
            telemetry.addData("rr", rr.getCurrentPosition());
            telemetry.update();
        }
        setMotorsPower(0, 0, 0, 0);
    }

    public void setMotorsPower(double lfm, double rfm, double lrm, double rrm) {
        lf.setPower(lfm);
        rf.setPower(rfm);
        lr.setPower(lrm);
        rr.setPower(rrm);
    }

    public void daTeCaMiBagPl(double power, int distance) {
        if (distance > 0) {
            strafeLeft(power, distance);
        } else {
            strafeRight(power, -distance);
        }
    }

    public void runBitch(double power, int distance) {
        if (distance > 0) {
            mergiFata(power, distance);
        } else {
            mergiSpate(power, -distance);
        }
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double delta = angles.firstAngle - lastAngles.firstAngle;

        if (delta < -180) {
            delta += 360;
        } else if (delta > 180) {
            delta -= 360;
        }

        global += delta;
        lastAngles = angles;

        return -global;
    }

    public void invartaTeAnPl(double power, int grade) {
        double left, right;
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        global = 0;

        if (grade < 0) {
            left = power;
            right = -power;

        } else if (grade > 0) {
            left = -power;
            right = power;
        } else return;

        lf.setPower(left);
        rf.setPower(right);
        lr.setPower(left);

        rr.setPower(right);
        double p;
        int a = 12;
        if (a == 1) {
            p = 0.05;
        } else {
            p = 0.2;
        }

        if (grade < 0) {
            while (getAngle() == 0) {
                telemetry.addData("angle", -getAngle());
                telemetry.update();
            }
            while (opModeIsActive() && -getAngle() > grade) {
                telemetry.addData("unghi", -getAngle());
                telemetry.addData("a", (grade - grade / 3.));
                telemetry.update();

                if (-getAngle() < (grade - grade / 3.)) {
                    lf.setPower(p);
                    rf.setPower(-p);
                    lr.setPower(p);
                    rr.setPower(-p);
                }
            }
            while (opModeIsActive() && -getAngle() < grade) {
                lf.setPower(-p);
                rf.setPower(p);
                lr.setPower(-p);
                rr.setPower(p);
            }
        } else {
            while (opModeIsActive() && -getAngle() <= grade) {
                telemetry.addData("unghi", getAngle());
                telemetry.addData("lf", lf.getPower());
                telemetry.addData("rf", rf.getPower());
                telemetry.addData("lr", lr.getPower());
                telemetry.addData("rr", rr.getPower());
                telemetry.update();

                if (-getAngle() > (grade - (grade / 3.))) {
                    lf.setPower(-p);
                    rf.setPower(p);
                    lr.setPower(-p);
                    rr.setPower(p);
                }
            }
            while (opModeIsActive() && -getAngle() > grade) {
                lf.setPower(p);
                rf.setPower(-p);
                lr.setPower(p);
                rr.setPower(-p);
            }
        }
        setMotorsPower(0, 0, 0, 0);
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        global = 0;
    }

    public void mergiAnPulea(double lfp, double rfp, double lrp, double rrp,
                             int lfmD, int rfmD, int lrmD, int rrmD) {
        int dist1 = lf.getCurrentPosition() + (lfmD * conversion);
        int dist2 = rf.getCurrentPosition() + (rfmD * conversion);
        int dist3 = lr.getCurrentPosition() + (lrmD * conversion);
        int dist4 = rr.getCurrentPosition() + (rrmD * conversion);
        while (opModeIsActive() &&
                (lf.getCurrentPosition() != dist1 || rf.getCurrentPosition() != dist2 || lr.getCurrentPosition() != dist3 || rr.getCurrentPosition() != dist4) &&
                (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle < 89)
        ) {
            lf.setPower(lfp);
            rf.setPower(rfp);
            lr.setPower(lrp);
            rr.setPower(rrp);
        }
        while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle < 90) {
            setMotorsPower(-0.05, 0.05, -0.05, 0.05);
        }
        setMotorsPower(0, 0, 0, 0);
        sleep(1000);
    }

    public void roteste(double grade) {
        double l, r, p = 0.2;

        if (grade < 0) {
            l = p;
            r = -p;

        } else if (grade > 0) {
            l = -p;
            r = p;
        } else return;


        if(grade > 0){
            while (opModeIsActive() && getAngle() <= grade) {
                setMotorsPower(l, r, l, r);
                telemetry.addData("grade", getAngle());
                telemetry.update();
            }
        }
        else{
            while (opModeIsActive() && getAngle() > grade) {
                setMotorsPower(l, r, l, r);
                telemetry.addData("grade", getAngle());
                telemetry.update();
            }
        }

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        global = 0;

        setMotorsPower(0,0,0,0);
        sleep(1500);

        // telemetry.addData("asd", grade);
        // if(grade > 0){
        // while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle < grade) {
        //     setMotorsPower(l, r, l, r);
        //     telemetry.addData("grade", getAngle());
        // telemetry.update();

        // }
        // }
        // else if(grade < 0){
        //     telemetry.addData("g", getAngle());
        //     telemetry.update();

        //     while (opModeIsActive() && getAngle() > grade) {
        //         setMotorsPower(l, r, l, r);
        //         telemetry.addData("grade", getAngle());
        //         telemetry.update();
        //         // System.out.println(getAngle());
        //     }
        // }
        // else return;

    }
}





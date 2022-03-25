package org.firstinspires.ftc.teamcode.Matei.hanga;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class askdj extends LinearOpMode {
    public DcMotor lf = null, rf = null, lr = null, rr = null;
    HardwareMap Hmap = null;

    //    pt encodere
    public static final double ticks = 145.6;
    public static final double diameter = 4.125;
    public static final double circuferenta = Math.PI * diameter;
    public static final double cpi = ticks / circuferenta;
    public static final double bias = 0.496;
    public static final int conversion = (int) (cpi * bias);

    public static final double InchInCm = 2.54;
    //    pt gyro
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double global = 0;

    public askdj() {
    }

    public void init(HardwareMap hMap) {
        Hmap = hMap;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        lf = hardwareMap.dcMotor.get("front_left_motor");
        rf = hardwareMap.dcMotor.get("front_right_motor");
        lr = hardwareMap.dcMotor.get("back_left_motor");
        rr = hardwareMap.dcMotor.get("back_right_motor");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);


        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = false;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    private void mergiFata(double power, int distance) {

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

    double diametru =64;
    double raza= diametru / 2;
    public void deplasare_encoder(int lm1, int lm2, int rm1, int rm2)
    {
        oprire_encoder();

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setTargetPosition(lm1);
        rf.setTargetPosition(lm2);
        lr.setTargetPosition(rm1);
        rr.setTargetPosition(rm2);
    }

    public void rotire_poz_dr_neg_st(double alfa)
    {
        oprire_encoder();

        double  distanta =( 2 * 3.14 * raza * (alfa / 360.)) / 2.54;

        double circumference = 3.14 * 3.937;
        double rotationNeeded = distanta/circumference;
        int encoderDrivingTarget = (int)(rotationNeeded * 1120);

        deplasare_encoder(encoderDrivingTarget, encoderDrivingTarget, encoderDrivingTarget, encoderDrivingTarget);

        lf.setPower(0.1);
        rf.setPower(0.1);
        lr.setPower(0.1);
        rr.setPower(0.1);

        while(lf.isBusy()||rf.isBusy()||lr.isBusy()||rr.isBusy())
        {
            telemetry.addData("Parcurcand distanta",distanta);
            telemetry.addData("[*]Distanta Calcul", "Running to %7d :%7d", encoderDrivingTarget, encoderDrivingTarget);
            telemetry.addData("[**]Distanta spate", "Running at %7d :%7d", lr.getCurrentPosition(), rr.getCurrentPosition());
            telemetry.addData("[***]Distanta fata", "Running at %7d :%7d", lf.getCurrentPosition(), rf.getCurrentPosition());

            telemetry.update();

        }
        //oprire_encoder();


    }

    public void oprire_encoder()
    {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //deplasare(0,0,0,0) ;
        // Set power to 0
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

        if (grade < 0) {
            while (getAngle() == 0) {
            }
            while (opModeIsActive() && getAngle() > grade) {
                telemetry.addData("unghi", getAngle());
                telemetry.addData("a", (grade - grade / 3.));
                telemetry.update();

                if (getAngle() < (grade - grade / 3.)) {
                    lf.setPower(.009);
                    rf.setPower(-.009);
                    lr.setPower(0.009);
                    rr.setPower(-.009);
                }
            }
            while (opModeIsActive() && getAngle() < grade) {
                lf.setPower(-.009);
                rf.setPower(.009);
                lr.setPower(-.009);
                rr.setPower(.009);
            }
        } else {
            while (opModeIsActive() && getAngle() < grade) {
                telemetry.addData("unghi", getAngle());

                telemetry.update();

                if(getAngle() > grade ){
                    lf.setPower(0);
                    rf.setPower(0);
                    lr.setPower(0);
                    rr.setPower(0);
                }
                if (getAngle() > (grade - (grade / 3.))) {
                    lf.setPower(-.009);
                    rf.setPower(.009);
                    lr.setPower(-0.009);
                    rr.setPower(.009);
                }
            }
            while (opModeIsActive() && getAngle() > grade) {
                lf.setPower(.009);
                rf.setPower(-.009);
                lr.setPower(0.009);
                rr.setPower(-.009);
            }
        }
        telemetry.addData("gata", getAngle());
        telemetry.update();

        setMotorsPower(0, 0, 0, 0);

        sleep(500);
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        global = 0;
    }
    public void mergiAnPuleaCM(double p1, double p2, double p3, double p4, int lfmD, int rfmD, int lrmD, int rrmD) {

        lfmD *= InchInCm;
        rfmD *= InchInCm;
        lrmD *= InchInCm;
        rrmD *= InchInCm;

        int dist1 = lf.getCurrentPosition() + (lfmD * conversion);
        int dist2 = rf.getCurrentPosition() + (rfmD * conversion);
        int dist3 = lr.getCurrentPosition() + (lrmD * conversion);
        int dist4 = rr.getCurrentPosition() + (rrmD * conversion);
        while (opModeIsActive() &&
                (lf.getCurrentPosition() != dist1 || rf.getCurrentPosition() != dist2 || lr.getCurrentPosition() != dist3 || rr.getCurrentPosition() != dist4) &&
                (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle < 89)
        ) {
            lf.setPower(p1);
            rf.setPower(p2);
            lr.setPower(p3);
            rr.setPower(p4);
        }
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle < 90){
            setMotorsPower(-0.005, 0.005, -0.005, 0.005);
        }
        setMotorsPower(0, 0, 0, 0);
    }
    public void mergiAnPulea(double p1, double p2, double p3, double p4, int lfmD, int rfmD, int lrmD, int rrmD) {
        int dist1 = lf.getCurrentPosition() + (lfmD * conversion);
        int dist2 = rf.getCurrentPosition() + (rfmD * conversion);
        int dist3 = lr.getCurrentPosition() + (lrmD * conversion);
        int dist4 = rr.getCurrentPosition() + (rrmD * conversion);
        while (opModeIsActive() &&
                (lf.getCurrentPosition() != dist1 || rf.getCurrentPosition() != dist2 || lr.getCurrentPosition() != dist3 || rr.getCurrentPosition() != dist4) &&
                (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle < 89)
        ) {
            lf.setPower(p1);
            rf.setPower(p2);
            lr.setPower(p3);
            rr.setPower(p4);
        }
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle < 90){
            setMotorsPower(-0.005, 0.005, -0.005, 0.005);
        }
        setMotorsPower(0, 0, 0, 0);
    }
}
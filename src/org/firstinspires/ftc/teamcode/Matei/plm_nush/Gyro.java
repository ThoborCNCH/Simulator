// package org.firstinspires.ftc.teamcode.Matei.plm_nush;
//
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//// import org.firstinspires.ftc.teamcode.Encodere;
//
// import static java.lang.Thread.sleep;
//
// public class Gyro extends LinearOpMode {
//     private final DcMotor[] motors;
//     private BNO055IMU imu;
//     private Orientation lastAngles = new Orientation();
//     private double global = 0, correction;
//
//     public Gyro(DcMotor[] motors) {
//         this.motors = motors;
//     }
//
//     public void init(HardwareMap hardwareMap) {
//         imu = hardwareMap.get(BNO055IMU.class, "imu");
//         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//         parameters.loggingEnabled = false;
//         parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//         parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//
//         imu.initialize(parameters);
//     }
//
//     private double getAngle() {
//         Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//         double delta = angles.firstAngle - lastAngles.firstAngle;
//
//         if (delta < -180) {
//             delta += 360;
//         } else if (delta > 180) {
//             delta -= 360;
//         }
//
//         global += delta;
//         lastAngles = angles;
//
//         return global;
//     }
//
//     public double checkDirection() {
//         double correction, angle = getAngle(), gain = 0.1;
//
//         if (angle == 0) {
//             correction = 0;
//         } else {
//             correction = -angle;
//         }
//
//         correction *= gain;
//
//         return correction;
//     }
//
//     private void resetAngle() {
//         lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//         global = 0;
//     }
//
//     public void rotire(int grade, double power) {
//         double lfm, lrm, rfm, rrm;
//         Encodere encodere = new Encodere(motors);
//         resetAngle();
//
//         encodere.runUsingEncoders();
//
//         if (grade < 0) {
//             lfm = power;
//             lrm = power;
//             rfm = -power;
//             rrm = -power;
//         } else if (grade > 0) {
//             lfm = -power;
//             lrm = -power;
//             rfm = power;
//             rrm = power;
//         } else return;
//
//         encodere.setMotorsPower(lfm, lrm, rfm, rrm);
//
//         if (grade < 0) {
//             while ( getAngle() == 0) {
//             }
//             while ( getAngle() > grade) {
//             }
//         } else {
//             while (getAngle() < grade) {
//             }
//         }
//
//
// //        double leftPower, rightPower, leftPowerSlow, rightPowerSlow, fast;
// //        Encodere encodere = new Encodere(motors);
// //
// //        encodere.runUsingEncoders();
// //
// //        resetAngle();
// //
// //        if (grade < 0) {
// //            leftPower = -power;
// //            rightPower = power;
// //
// //            leftPowerSlow = -0.15;
// //            rightPowerSlow = 0.15;
// //            fast = grade + 30;
// //
// //        } else if (grade > 0) {
// //            leftPower = power;
// //            rightPower = -power;
// //
// //            leftPowerSlow = 0.15;
// //            rightPowerSlow = -0.15;
// //            fast = grade - 30;
// //        } else return;
// //
// //        encodere.setMotorsPower(leftPower, leftPower, rightPower, rightPower);
// //
// //        if (grade < 0) {
// //            while (getAngle() == 0) {}
// //
// //            while (getAngle() > grade) {
// //                telemetry.addData("1 imu heading", lastAngles.firstAngle);
// //                telemetry.addData("2 global heading", global);
// //                telemetry.addData("3 getangle", getAngle());
// //                telemetry.addData("4 degrees", grade);
// //                telemetry.addData("5 fast", fast);
// //                telemetry.update();
// //
// //                if (getAngle() > fast) {
// //                    encodere.setMotorsPower(leftPower, leftPower, rightPower, rightPower);
// //                } else if (getAngle() > grade) {
// //                    encodere.setMotorsPower(leftPowerSlow, leftPowerSlow, rightPowerSlow, rightPowerSlow);
// //                }
// //
// //            }
// //        } else {
// //            while (getAngle() < grade) {
// //                telemetry.addData("1 imu heading", lastAngles.firstAngle);
// //                telemetry.addData("2 global heading", global);
// //                telemetry.addData("3 getangle", getAngle());
// //                telemetry.addData("4 degrees", grade);
// //                telemetry.addData("5 degrees - 30", fast);
// //                telemetry.update();
// //                if (getAngle() < fast) {
// //                    encodere.setMotorsPower(leftPower, leftPower, rightPower, rightPower);
// //                } else if (getAngle() < grade) {
// //                    encodere.setMotorsPower(leftPowerSlow, leftPowerSlow, rightPowerSlow, rightPowerSlow);
// //                }
// //            }
// //        }
//
//         encodere.setMotorsPower(0, 0, 0, 0);
//         sleep(1000);
//         resetAngle();
//
//     }
//
//     @Override
//     public void runOpMode() throws InterruptedException {
//
//     }
// }

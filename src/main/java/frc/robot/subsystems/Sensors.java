package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;

import org.opencv.objdetect.QRCodeDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;

public class Sensors extends SubsystemBase {
    public Sensors() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        Thread m_visionThread = new Thread(
                () -> {
                    // Get the UsbCamera from CameraServer
                    UsbCamera camera = CameraServer.startAutomaticCapture();
                    QRCodeDetector detector = new QRCodeDetector();
                    // Set the resolution (find out resolution of camera once one is selected &
                    // change)
                    camera.setResolution(640, 480);

                    // Get a CvSink to capture Mats from the camera
                    CvSink cvSink = CameraServer.getVideo();
                    // Setup a CvSource to send images back to the Dashboard
                    CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

                    // reuse this Mat.
                    Mat mat = new Mat();

                    System.out.println("Initialized camera");

                    // This cannot be 'true'. The program will never exit if it is. This
                    // lets the robot stop this thread when restarting robot code or
                    // deploying.
                    while (!Thread.interrupted()) {
                        // Tell the CvSink to grab a frame from the camera and put it
                        // in the source mat. If there is an error notify the output.
                        if (cvSink.grabFrame(mat) == 0) {
                            // Send the output the error.
                            outputStream.notifyError(cvSink.getError());
                            // skip the rest of the current iteration
                            continue;
                        }
                        // Process image
                        List<String> decoded_info = new ArrayList<String>();
                        boolean hasDetections = detector.detectAndDecodeMulti(mat, decoded_info);
                        if (hasDetections) {
                            System.out.println(String.format("Found %d codes", decoded_info.size()));
                            if (decoded_info.size() != 1) {
                                System.out.println("Found multiple QR codes. Ignoring frame.");
                                continue;
                            }
                            String qr_code_text = decoded_info.get(0);
                            System.out.println(String.format("Found %s codes", qr_code_text));
                            if (qr_code_text.startsWith("tj2-battery")) {
                                // logging info to DS
                                System.out.println("Battery code: " + qr_code_text);
                                break;
                            }
                        }
                    }
                });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
    }

    @Override
    public void periodic() {

    }
}

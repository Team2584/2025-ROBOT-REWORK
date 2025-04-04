package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_USBCAM;

public class USBCamera extends SubsystemBase {

  private final UsbCamera camera1;
  private final UsbCamera climbCamera;
  private final VideoSink server;

  public USBCamera() {
    // Camera initialization
    camera1 = CameraServer.startAutomaticCapture(CONSTANTS_USBCAM.CAM01_ID);
    climbCamera = CameraServer.startAutomaticCapture(CONSTANTS_USBCAM.CAM02_ID);
    server = CameraServer.getServer();

    camera1.setResolution(CONSTANTS_USBCAM.RES_WIDTH, CONSTANTS_USBCAM.RES_HEIGHT);
    camera1.setFPS(CONSTANTS_USBCAM.FPS);

    climbCamera.setResolution(CONSTANTS_USBCAM.RES_WIDTH, CONSTANTS_USBCAM.RES_HEIGHT);
    climbCamera.setFPS(CONSTANTS_USBCAM.FPS);
  }

  public void setCameraClimb() {
    server.setSource(climbCamera);
  }
}

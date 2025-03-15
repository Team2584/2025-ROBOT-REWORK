// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import frc.robot.CONSTANTS;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS.CONSTANTS_USBCAM;
import edu.wpi.first.wpilibj2.command.Command;

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
    server.setSource(climbCamera);
  }


}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  Thread m_visionThread;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  
  private edu.wpi.first.wpilibj.util.Color kNoteTarget = new edu.wpi.first.wpilibj.util.Color(53,37,9);
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  // camera dimensions
  private int width = 640;
  private int height = 480;
  // rectangle dimensions
  private double pct_width = .3;
  private double pct_height = .1;
  double midX = width / 2;
  double midY = height / 2;
  double startX = midX - width * pct_width;
  double startY = midY - height * pct_height;
  double endX = midX + width * pct_width;
  double endY = midY + height * pct_height;

  // polygon dimensions
  // top left
  double p1x = midX - width * pct_width;
  double p1y = midY - height * pct_height;

  // top right
  double p2x = midX - width * pct_width;
  double p2y = midY + height * pct_height;

  // bottom left
  double p3x = midX + width * pct_width;
  double p3y = midY + height * pct_height;

  // bottom right
  double p4x = midX + width * pct_width;
  double p4y = midY - height * pct_height;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    m_colorMatcher.addColorMatch(kNoteTarget);

    m_colorMatcher.setConfidenceThreshold(0.72);

    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture(1);
    
    m_visionThread = new Thread(
        () -> {

          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture(0);
          // Set the resolution
          camera.setResolution(width, height);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Rectangle", width, height);

          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();

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
            // Put a rectangle on the image
            Imgproc.rectangle(
                mat, new Point(startX, startY), new Point(endX, endY), new Scalar(0, 255, 0), 1);

            Point p1 = new Point(p1x, p1y);
            Point p2 = new Point(p2x, p2y);
            Point p3 = new Point(p3x, p3y);
            Point p4 = new Point(p4x, p4y);

            double angle1 = 30;

            List<MatOfPoint> list1 = new ArrayList<MatOfPoint>();
            list1.add(new MatOfPoint(
                rotatePoint(p1, angle1, midX, midY),
                rotatePoint(p2, angle1, midX, midY),
                rotatePoint(p3, angle1, midX, midY),
                rotatePoint(p4, angle1, midX, midY)));

            Imgproc.polylines(
                mat, list1, true, new Scalar(0, 255, 255), 1);

                double angle2 = -30;
            List<MatOfPoint> list2 = new ArrayList<MatOfPoint>();
            list2.add(new MatOfPoint(
                rotatePoint(p1, angle2, midX, midY),
                rotatePoint(p2, angle2, midX, midY),
                rotatePoint(p3, angle2, midX, midY),
                rotatePoint(p4, angle2, midX, midY)));

            Imgproc.polylines(
                mat, list2, true, new Scalar(0, 255, 255), 1);

            // Give the output stream a new image to display
            outputStream.putFrame(mat);
          }
        });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  public static Point rotatePoint(Point point, double angle, double midx, double midy) {
    double oldX = point.x;
    double oldY = point.y;

    double oldCx = oldX - midx; // move middle to (0,0)
    double oldCy = oldY - midy; // move middle to (0,0)

    double angleRadians = Math.toRadians(angle);

    double angleSin = Math.sin(angleRadians);
    double angleCos = Math.cos(angleRadians);

    double newCx = oldCx * angleCos - oldCy * angleSin;
    double newCy = oldCx * angleSin + oldCy * angleCos;

    double newX = newCx + midx; // move back
    double newY = newCy + midy; // move back

    return new Point((int) newX, (int) newY);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Angle", m_robotContainer.m_drivetrain.getGyroAngle());
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    edu.wpi.first.wpilibj.util.Color detectedColor = m_colorSensor.getColor();

    int IR = m_colorSensor.getIR();

    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);

    if (match != null && match.color == kNoteTarget) {
      colorString = "Orange";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red",detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putString("Detected Color", colorString);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.m_drivetrain.calibrateGyro();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}

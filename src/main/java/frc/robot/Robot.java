// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public TalonFX angle = new TalonFX(10);
  public TalonFX velo = new TalonFX(2);

  public Robot() {
    super(0.005);
  }

  private void configureDriveMotor() {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0.kP = 20.0;
    driveConfig.Slot0.kI = 0;
    driveConfig.Slot0.kD = 0;

    driveConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 1;
    driveConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = 2;

    driveConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    driveConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    driveConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
    driveConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

    var driveConfig2 = new TalonFXConfiguration();
    driveConfig2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig2.Slot0.kP = 20.0;
    driveConfig2.Slot0.kI = 0;
    driveConfig2.Slot0.kD = 0;

    driveConfig2.HardwareLimitSwitch.ForwardLimitRemoteSensorID = angle.getDeviceID();
    driveConfig2.HardwareLimitSwitch.ReverseLimitRemoteSensorID = angle.getDeviceID();

    driveConfig2.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteTalonFX;
    driveConfig2.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteTalonFX;

    driveConfig2.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
    driveConfig2.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

    angle.getConfigurator().apply(driveConfig);
    velo.getConfigurator().apply(driveConfig2);
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("PercentOut1", 0);
    SmartDashboard.putNumber("PercentOut2", 0);

    configureDriveMotor();
  }

  public double getPercentOut1() {
    return NetworkTableInstance
      .getDefault()
      .getTable("/SmartDashboard")
      .getEntry("PercentOut1")
      .getDouble(0.0);
  }

    public double getPercentOut2() {
    return NetworkTableInstance
      .getDefault()
      .getTable("/SmartDashboard")
      .getEntry("PercentOut2")
      .getDouble(0.0);
  }

 

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("RPM 1", angle.getRotorVelocity().getValue() * 60);
    SmartDashboard.putNumber("RPM 2", velo.getRotorVelocity().getValue() * 60);


    var request1 = new DutyCycleOut(getPercentOut1());
    var request2 = new DutyCycleOut(getPercentOut2());

    angle.setControl(request1);
    velo.setControl(request2);
    }

    // var request1 = new DutyCycleOut(getPercentOut());
    // motor.setControl(request1);

    // SmartDashboard.putNumber("RPS", motor.getRotorVelocity().getValue());
    // SmartDashboard.putNumber("RPM 1", motor.getRotorVelocity().getValue() * 60);
  }
 



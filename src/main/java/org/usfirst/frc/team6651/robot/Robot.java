/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6651.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	// SmartDashBoard Init
	SmartDashboard SmartDashBoard;

	// UltraSound Init
	AnalogInput ai;

	// Encoders Init
	Encoder enc1, enc2, enc3, enc4;

	// Port init for Relays - RGB LEDs
	Relay LEDRed, LEDGreen, LEDBlue;
	int RedPort = 0;
	int GreenPort = 2;
	int BluePort = 3;
	
	// Mecanum Drive Init and motor controllers
	public static MecanumDrive DTMec;
	WPI_TalonSRX talon10;
	WPI_TalonSRX talon11;
	WPI_TalonSRX talon12;
	WPI_TalonSRX talon13;
	
	// Gyro Init
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	double angle;
	int resetGyroId = 2;
	
	// Joystick Init
	Joystick PS4 = new Joystick(0);
	int butterflyButtonId = 1;
	
	// MaxPower - Use less than 1 to slow down the robot
	double MAXPOWER=0.5;
	
	// Pneumatic Init
	Compressor c;
	DoubleSolenoid butterflySolenoid;
	boolean changeOfState = true;
	DoubleSolenoid.Value UP=DoubleSolenoid.Value.kForward, DOWN=DoubleSolenoid.Value.kReverse;
	DoubleSolenoid.Value butterflyState = DOWN;

	@Override
	public void robotInit() {
		// UltraSount Init in port 2
		ai = new AnalogInput(2);

		// Right side Controllers
		talon10 = new WPI_TalonSRX(10);
		talon12 = new WPI_TalonSRX(12);

		// Left side Controllers
		talon11 = new WPI_TalonSRX(11);
		talon13 = new WPI_TalonSRX(13);

		// DriveTrain Init
		DTMec = new MecanumDrive(talon10, talon11, talon12, talon13);
		
		// Pneumatic Init
		c = new Compressor(0);
		c.setClosedLoopControl(true);  // Start compressor control
		butterflySolenoid = new DoubleSolenoid(0, 1);
		butterflySolenoid.set(butterflyState);

		// LED Strip Init
		LEDRed = new Relay(RedPort);
		LEDRed.set(Relay.Value.kForward);
		LEDRed.set(Relay.Value.kOn);

		LEDGreen = new Relay(GreenPort);
		LEDGreen.set(Relay.Value.kForward);
		LEDGreen.set(Relay.Value.kOff);

		LEDBlue = new Relay(BluePort);
		LEDBlue.set(Relay.Value.kForward);
		LEDBlue.set(Relay.Value.kOff);

		// GYRO Init
		gyro.reset();
		// gyro.calibrate(); // Calibrate once in a while

		// Encoders Init
		enc1 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		enc2 = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		enc3 = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		enc4 = new Encoder(6, 7, false, Encoder.EncodingType.k4X);

		// Encoders Init
		enc1.reset();
		enc2.reset();
		enc3.reset();
		enc4.reset();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		// LED Init
		LEDRed.set(Relay.Value.kOff);
		LEDGreen.set(Relay.Value.kOff);
		LEDBlue.set(Relay.Value.kOn);

		// Encoders Init
		enc1.reset();
		enc2.reset();
		enc3.reset();
		enc4.reset();

		// GYRO Init
		gyro.reset();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		autonomous_rangecalibrate();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		// Joystick axis - definition
		int X_axis = 1, Y_axis = 0, Z_axis = 2;
		double power=1*MAXPOWER; // Power is used for POV speed
		double forward = PS4.getRawAxis(X_axis)*MAXPOWER; 
		double slide = PS4.getRawAxis(Y_axis)*MAXPOWER; 
		double turn = PS4.getRawAxis(Z_axis)*MAXPOWER;
		int POV = PS4.getPOV();
		
		// Reset Gyro
		if (PS4.getRawButton(resetGyroId) == true) 
		{
			gyro.reset();
			enc1.reset();
			enc2.reset();
			enc3.reset();
			enc4.reset();
		}

		// Input from Gyro
		angle = gyro.getAngle();

		// Test to check mode (Butterfly is DOWN, Tank Mode is UP)
		if (butterflyState == DOWN)
		{
			turn = slide;
			slide = 0;
		}
		else // Tank mode
		{
			// Cases for POV
			if (POV != -1) System.out.println("POV " + POV);
			switch(POV)
				{
					case 0:
						forward = -power; 
						turn = 0;
						slide = 0;
						break;
					case 45:
						forward = -power; 
						turn = 0;
						slide = power;
						break;
					case 90:
						forward = 0; 
						turn = 0;
						slide = power;
						break;
					case 135:
						forward = power; 
						turn = 0;
						slide = power;
						break;
					case 180:
						forward = power; 
						turn = 0;
						slide = 0;
						break;
					case 225:
						forward = power; 
						turn = 0;
						slide = -power;
						break;
					case 270:
						forward = 0; 
						turn = 0;
						slide = -power;
						break;
					case 315:
						forward = -power; 
						turn = 0;
						slide = -power;
						break;
					case -1:
						break;
				}
		}
		DTMec.driveCartesian(forward, slide, -turn);
		
		// Change mode between Butterfly and Tank
		if (PS4.getRawButton(butterflyButtonId) == true && changeOfState == true) 
		{
			if (butterflyState == DOWN) 
			{
				butterflyState = UP;
				LEDRed.set(Relay.Value.kOff);
				LEDGreen.set(Relay.Value.kOn);
				LEDBlue.set(Relay.Value.kOff);
			}
			else 						
			{
				butterflyState = DOWN;
				LEDRed.set(Relay.Value.kOn);
				LEDGreen.set(Relay.Value.kOff);
				LEDBlue.set(Relay.Value.kOff);
			}
			changeOfState = false;
			butterflySolenoid.set(butterflyState);
		}
		
		// Reset state for driving mode
		// It will change states only when button has been released and pushed again
		if (PS4.getRawButton(butterflyButtonId) == false && changeOfState == false)  
			changeOfState = true;

		// Update SmartDashBoard
		updateDashboard();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	public void autonomous_1() {
		int average_enc = (-enc1.get()-enc2.get()-enc3.get()-enc4.get())/4;
		angle = gyro.getAngle();
		if (average_enc < 500)
		{
			DTMec.driveCartesian(0.3, 0, -0, angle);
		}
		else DTMec.driveCartesian(0, 0, -0, angle);
		
		
	}
	
	public void autonomous_rangecalibrate() {
		DTMec.driveCartesian(0, 0, -0);
		updateDashboard();
	}

	public void updateDashboard(){
		double distance_travelled = get_encoder_distance();
		double ultrasound = get_ultrasound_distance();
		SmartDashboard.putNumber("Distance Travelled: ", distance_travelled);
		SmartDashboard.putNumber("Ultrasound: ", ultrasound);
		SmartDashboard.putNumber("Angle: ", gyro.getAngle());
		SmartDashboard.putNumber("enc1: ", enc1.get());
		SmartDashboard.putNumber("enc2: ", enc2.get());
		SmartDashboard.putNumber("enc3: ", enc3.get());
		SmartDashboard.putNumber("enc4: ", enc4.get());
	}

	public double get_encoder_distance(){
		double encoder;

		encoder = (-enc1.get()+enc3.get()-enc4.get())/3;

		return encoder*7*18/1400;
	}

	public double get_ultrasound_distance(){
		double ultrasound;

		ultrasound = ai.getAverageVoltage()*24/0.286;

		return ultrasound;
	}

}


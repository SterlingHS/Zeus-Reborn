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

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	
	public static MecanumDrive DTMec;
	
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	double angle;
	double kp=0.004;
	double tick_per_degree=0.000144114;
	
	Joystick PS4 = new Joystick(0);
	int butterflyButtonId = 1;
	
	double MAXPOWER=0.75;
	WPI_TalonSRX talon10;
	WPI_TalonSRX talon11;
	WPI_TalonSRX talon12;
	WPI_TalonSRX talon13;
	
	Compressor c;
	DoubleSolenoid butterflySolenoid;
	boolean changeOfState = true;
	DoubleSolenoid.Value UP=DoubleSolenoid.Value.kForward, DOWN=DoubleSolenoid.Value.kReverse;
	DoubleSolenoid.Value butterflyState = DOWN;
	
	@Override
	public void robotInit() {
		talon10 = new WPI_TalonSRX(10);
		talon12 = new WPI_TalonSRX(12);

		talon11 = new WPI_TalonSRX(11);
		talon13 = new WPI_TalonSRX(13);

		DTMec = new MecanumDrive(talon10, talon11, talon12, talon13);
		
		c = new Compressor(0);
		c.setClosedLoopControl(true);  // Start compressor control
		
		butterflySolenoid = new DoubleSolenoid(0, 1);
		butterflySolenoid.set(butterflyState);

		// Calibrate once in a while
		// gyro.calibrate();
		gyro.reset();

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
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		int X_axis = 1, Y_axis = 0, Z_axis = 2;
		double power=0.8*MAXPOWER;
		double forward = PS4.getRawAxis(X_axis)*MAXPOWER; 
		double slide = PS4.getRawAxis(Y_axis)*MAXPOWER; 
		double turn = PS4.getRawAxis(Z_axis)*MAXPOWER;
		int POV = PS4.getPOV();
		angle = gyro.getAngle();
		// System.out.println("Angle at: " + angle + "    The angle: " + (int)(angle/tick_per_degree));
		
		if (butterflyState == DOWN)
		{
			turn = slide;
			slide = 0;
		}
		else {
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
		DTMec.driveCartesian(forward, -slide, -turn);
		
		if (PS4.getRawButton(butterflyButtonId) == true && changeOfState == true) 
		{
			if (butterflyState == DOWN) butterflyState = UP;
			else 						butterflyState = DOWN;
			changeOfState = false;
			butterflySolenoid.set(butterflyState);
		}
		
		//  Reset state for driving mode
		if (PS4.getRawButton(butterflyButtonId) == false && changeOfState == false)  changeOfState = true;


	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}

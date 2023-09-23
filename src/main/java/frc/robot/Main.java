// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what
 * you are doing, do not modify this file except to change the parameter class
 * to the startRobot
 * call.
 */
public final class Main {
    private Main() {
    }

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>
     * If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {

        // Test LUT OOB interpolation
		// var armDLUT = new InterpLUT();
		// armDLUT.add(0, 0.0);
		// armDLUT.add(1, 1.0);
		// armDLUT.add(2, 2.0);
		// armDLUT.createLUT();
        // System.out.println(armDLUT);
        // System.out.println("-1  " + armDLUT.get(-1.0));
        // System.out.println("0.0 " + armDLUT.get(0.0));
        // System.out.println("0.5 " + armDLUT.get(0.5));
        // System.out.println("1.0 " + armDLUT.get(1));
        // System.out.println("2.0 " + armDLUT.get(2));
        // System.out.println("3.0 " + armDLUT.get(3));
        // System.out.println("4.0 " + armDLUT.get(4));

        // var armDLUT = new InterpLUT();
		// armDLUT.add(0, 500);
		// armDLUT.add(48, 100);
		// armDLUT.createLUT();
        // System.out.println("0.0 " + armDLUT.get(0));
        // System.out.println("48.0 " + armDLUT.get(49.1));
        RobotBase.startRobot(Robot::new);
    }
}

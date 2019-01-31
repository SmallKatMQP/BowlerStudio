package com.neuronrobotics.smallkat;

import com.neuronrobotics.bowlerstudio.BowlerStudio;
import com.neuronrobotics.sdk.addons.kinematics.DHChain;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import java.util.ArrayList;

import com.neuronrobotics.sdk.addons.kinematics.DHChain;
import com.neuronrobotics.sdk.addons.kinematics.DHLink;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.common.Log;
import Jama.Matrix;

public class JavaIKModel implements DhInverseSolver {

	@Override
	public double[] inverseKinematics(TransformNR target, double[] jointSpaceVector, DHChain chain) {
		System.out.println("My IK");
        try {
            ArrayList<DHLink> links = chain.getLinks();
            // THis is the jacobian for the given configuration
            //Matrix jacobian =  chain.getJacobian(jointSpaceVector);
            Matrix taskSpacMatrix = target.getMatrixTransform();

            int linkNum = jointSpaceVector.length;

            double L1x = 165;
            double L1y = -38.2605;
            double L1z = 0;
            
            
            double x = target.getX();
            double y = target.getY();
            double z = target.getZ();
            
//            double x = 165;
//            double y = -40;
//            double z = -150;
            
            //calculate link vector
//            x = (0-L1x) + x;
//            y = (0-L1y) + y;
//            z = (0-L1z) + z;

            System.out.println("x: " + x);
            System.out.println("y: " + y);
            System.out.println("z: " + z);

            double ang = 0.5236;


            //System.out.println(target.getRotation().toString());

            //double l1_d = links.get(0).getR();
            //double l2_d = links.get(1).getR();
            //double l3_d = links.get(2).getR();
            //double l4_d = links.get(3).getR();
            
            double l1_d = 0;
            double l2_d = 92;
            double l3_d = 75;
            double l4_d = 71.03;

            System.out.println("L1: " + l1_d);
            System.out.println("L2: " + l2_d);
            System.out.println("L3: " + l3_d);
            System.out.println("L4: " + l4_d);
            
            
            double[] inv = new double[linkNum];

            double theta1 = Math.atan(y / Math.abs(z));

            double r1 = Math.sqrt(Math.pow(z, 2) + Math.pow(y,2));
            double x1 = r1;
            double y1 = x;
            double Px = x1 - l4_d * Math.sin(ang);
            double Py = y1 - l4_d * Math.cos(ang);
            // Make below negative to switch to other angle
            double theta3_1 = -1* (Math.acos(((Math.pow(Px, 2) + Math.pow(Py,2)) - (Math.pow(l2_d,2) + Math.pow(l3_d, 2))) / (2 * l2_d * l3_d)));
            double theta3_2 = theta3_1 * -1;

            double B = Math.atan(Py / Px);
            double Y = Math.acos((Math.pow(Px, 2) + Math.pow(Py, 2) + Math.pow(l2_d, 2) - Math.pow(l3_d, 2)) / (2 * l2_d * Math.sqrt(Math.pow(Px, 2) + Math.pow(Py, 2))));

            double theta2_1 = B + Y;
            double theta2_2 = B - Y;

            double theta4_1 = (Math.PI / 2 - ang) - (theta2_1 + theta3_1);
            double theta4_2 = (Math.PI / 2 - ang) - (theta2_2 + theta3_2);

			//theta1 = 90;
			//theta2_1 = 0;
			//theta3_1 = 0;
			//theta4_1 = 0;
			
            System.out.println(theta1);
            System.out.println(theta2_1);
            System.out.println(theta3_1);
            System.out.println(theta4_1);
		
			
            //inv[0] = theta1; inv[1] = theta2_1; inv[2] = theta3_1; inv[3] = theta4_1;
            inv[0] = Math.toDegrees(theta1);
            inv[1] = Math.toDegrees(theta2_1);
            inv[2] = Math.toDegrees(theta3_1);
            inv[3] = Math.toDegrees(theta4_1);

            System.out.println("\r\n\r\nJoint Vector = " + inv + "\r\n\r\n");

            return inv;
        } catch (Throwable t) {
            BowlerStudio.printStackTrace(t);
            return null;
        }
	}

}

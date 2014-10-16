package tjs.robot;

import static robocode.util.Utils.normalRelativeAngle;
import static robocode.util.Utils.normalRelativeAngleDegrees;

import java.awt.Color;
import java.awt.Graphics2D;

import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.Robot;
import robocode.RobotDeathEvent;
import robocode.ScannedRobotEvent;

/**
 * Robot to go sit in a corner and fire towards the thick of combat
 * 
 * Based on Corners by Mathew A. Nelson and Flemming N. Larsen
 * 
 * @author Ted Snider
 *
 */
public class Sulker extends Robot {
	int others; // Number of other robots in the game
	double[][] corners;				//  Set corners array {X,Y, left corner, right corner}
									//  array coordinates offset for width of bot
	double[] targetCorner = {18.0, 18.0, 1.0, 3.0};
	boolean isAtWall = false;		// count wall collisions
	boolean isInCorner = false;		// note corner location
	double radarIncrement = 360;		// sweep field
	boolean unlimberGun = false;

	public void run() {
		double[][] setCorners = {{18.0, 18.0, 1.0, 3.0},
								 {18.0, getBattleFieldHeight()-18.0, 2.0, 0.0},
								 {getBattleFieldWidth()-18.0, getBattleFieldHeight()-18.0, 3.0, 1.0},
								 {getBattleFieldWidth()-18.0, 18.0, 0.0, 3.0}};
		corners = setCorners;
		
		// 	Set colors
		setBodyColor(Color.black);
		setGunColor(Color.white);
		setRadarColor(Color.red);
		setBulletColor(Color.red);
		setScanColor(Color.white);

		// Save # of other bots
		others = getOthers();

		layCourse(plotNearestCorner(getX(), getY()));
		
		// uncouple the gun from the body
		unlimberGun = true;
		setAdjustGunForRobotTurn(true);
		// uncouple radar from gun
		setAdjustRadarForGunTurn(true);

		// Spin radar back and forth
		int direction = 1;
		double x = getX();
		double y = getY();
		// sets initial position and returns sweep angle
		radarIncrement = adjustRadarSweep(x, y);
		while (true) {
			out.println("Current radar heading: "+getRadarHeading());
			out.println("turning "+(direction*radarIncrement)+" degrees.");
			turnRadarRight(direction * radarIncrement);
			out.println("changing direction of radar sweep.");
			direction *= -1;
		}
	}
	
	/**
	 * onHitRobot
	 * 
	 * need to be able to resume moving toward a corner
	 * 
	 */
	public void onHitRobot(HitRobotEvent e) {
		out.println("Oh no! I've hit a robot. ("+getX()+", "+getY()+")");
		// If he's in front of us, set back up a bit.
		if (e.getBearing() > -90 && e.getBearing() < 90) {
			back(100);
		} else {
			ahead(100);
		}
		
		// if the robot is idle, go around
		layCourse(plotNextCorner(getX(), getY()));
	}
	
	public void onHitWall(HitWallEvent e) {
		out.println("I've hit a wall.");
	} 
	
	/**
	 * onRobotDeath: update other robot count variable for strategic adjustments
	 */
	public void onRobotDeath(RobotDeathEvent e) {
		others = getOthers();
	}

	/**
	 * onScannedRobot
	 * 
	 * 1. check heat
	 * 2. calculate shot power based in distance...weaker/faster for further targets
	 * 3. determine an amount to lead target based on current velocity
	 * 
	 * @param ScannedRobotEvent e - event pertaining to robot scanned
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		// No point if heat is > 0
		if (getGunHeat() == 0) {
			double robotDistance = e.getDistance();
			double bullet = 1.0;
			
			// lower power moves faster and easier to reach distant targets
			if (robotDistance > 200 || getEnergy() < 15) {
				bullet = 1.0;
			} else if (robotDistance > 50) {
				bullet = Math.min(2, getEnergy());
			} else {
				bullet = Math.min(3, getEnergy());
			}
			
			if (unlimberGun) {
				// calculate rough linear prediction targeting
				double absoluteBearing = toRadians(getHeading()) + e.getBearingRadians();
				double adjustment = (e.getVelocity() * Math.sin(e.getHeadingRadians() - absoluteBearing) / 13.0);
				turnGunRight(toDegrees(normalRelativeAngle(absoluteBearing - toRadians(getGunHeading()) + adjustment)));
			}
			
			// kill
			fire(bullet);
		}
	}

	public void onPaint(Graphics2D g) {
		//g.draw(new Arc2D.Double(arg0, arg1, arg2, arg3, arg4, arg5, arg6));
	}
	
	private void layCourse(double[] destination) {
		turnRight(destination[0]);
		ahead(destination[1]);
	}

	/**
	 * plotNearestCorner(double, double): calculates nearest corner from passed-in location
	 * 
	 * SIDE EFFECT: sets targetCorner array
	 * 
	 * @param x
	 * @param y
	 * @return double array of polar coordinates for course (angle, distance)
	 */
	private double[] plotNearestCorner(double x, double y) {
		double shortest = 999999;
		double current;
		for (int i = 0; i < 4; i++) {
			current = Math.sqrt((corners[i][0]-x)*(corners[i][0]-x) + (corners[i][1]-y)*(corners[i][1]-y));
			if (current < shortest) {
				shortest = current;
				targetCorner = corners[i];
			}
		}
		double newHeading = Math.atan2(targetCorner[0]-x,targetCorner[1]-y);
		double[] plot = {toDegrees(newHeading - toRadians(getHeading())), shortest};
		
		return plot;
	}

	/**
	 * plotNextCorner(double, double): calculates direction and distance of 
	 * next corner in corners array from the passed-in location
	 * 
	 * SIDE EFFECT: sets targetCorner array
	 * 
	 * @param x
	 * @param y
	 * @return double array of polar coordinates for course (angle, distance)
	 */
	private double[] plotNextCorner(double x, double y) {
		double[] nextCorner = corners[(int)targetCorner[2]];
		double[] plot = {toDegrees(Math.atan2(nextCorner[0]-x, nextCorner[1]-y)),
						 Math.sqrt((nextCorner[0]-x)*(nextCorner[0]-x)+(nextCorner[1]-y)*(nextCorner[1]-y))};
		return plot;
	}

	private void turnRadarToHeading(double bearing) {
		turnRadarLeft(normalRelativeAngleDegrees(bearing-getRadarHeading()));
	}

	private double plotRightCornerBearing(double x, double y) {
		double[] rightCorner = corners[(int)targetCorner[3]];
		double cornerBearing = toDegrees(Math.atan2(rightCorner[0]-x, rightCorner[1]-y));
		return cornerBearing;
	}

	private double plotLeftCornerBearing(double x, double y) {
		double[] leftCorner = corners[(int)targetCorner[2]];
		double cornerBearing = toDegrees(Math.atan2(leftCorner[0]-x, leftCorner[1]-y));
		return cornerBearing;
	}
	
	private double adjustRadarSweep(double x, double y) {
		double leftLimit = plotLeftCornerBearing(x, y);
		out.println("Left corner bearing: "+leftLimit);
		double rightLimit = plotRightCornerBearing(x, y);
		out.println("Right corner bearing: "+rightLimit);
		out.println("Current radar heading: "+getRadarHeading());
		
		turnRadarToHeading(leftLimit);
		out.println("Current radar heading: "+getRadarHeading());
		
		return normalRelativeAngleDegrees(rightLimit - getRadarHeading());
	}
	
	/**
	 * since this is a basic bot, I can only get bearing, etc., in degrees, 
	 * but my algorithms all seem to use radians.
	 * 
	 *  yay
	 *  
	 *  conversion is not exact, but close enough
	 *  
	 * @param degrees
	 * @return double - radian equivalent of the degree measure 
	 */
	private double toRadians(double degrees) {
		return degrees * 0.0174532925;
	}
	
	/**
	 * since this is a basic bot, I can only set bearing, etc., in degrees,
	 * but my algorithms all seem to use radians.
	 * 
	 * yay
	 * 
	 * conversion is not exact, but close enough
	 * 
	 * @param radians
	 * @return double - degree equivalent of the radian measure
	 */
	private double toDegrees(double radians) {
		return radians * 57.2957795;
	}
}

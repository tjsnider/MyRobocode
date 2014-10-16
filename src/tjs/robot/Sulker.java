package tjs.robot;

import static robocode.util.Utils.normalRelativeAngle;

import java.awt.Color;

import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.Robot;
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
	private double maxX; // right wall
	private double maxY; // top wall
	private double minX = 0.0; // left wall
	private double minY = 0.0; // bottom wall
	boolean isAtWall = false; // count wall collisions
	boolean isInCorner = false; // note corner location
	int radarIncrement = 360; // sweep field

	public void run() {
		// 	Set colors
		setBodyColor(Color.black);
		setGunColor(Color.red);
		setRadarColor(Color.white);
		setBulletColor(Color.black);
		setScanColor(Color.white);

		// Save # of other bots
		others = getOthers();

		// get boundries
		maxX = getBattleFieldWidth();
		maxY = getBattleFieldHeight();
		
		// uncouple the gun from the body
		setAdjustGunForRobotTurn(true);
		// uncouple radar from gun
		setAdjustRadarForGunTurn(true);

		turnTowardNearestCorner(getX(), getY());
		ahead(5000);
		
		// Spin radar back and forth
		while (true) {
			turnRadarLeft(radarIncrement);
			radarIncrement *= -1;
		}
	}
	
	private void turnTowardNearestCorner(double x, double y) {
		double nearX = maxX-x < x ? maxX-x : -x;
		double nearY = maxY-y < y ? maxY-y : -y;
		double newHeading = Math.atan(nearY/nearX);
		
		out.println("Heading toward ("+nearX+", "+nearY+") on bearing "+newHeading);
		
		turnRight(toDegrees(normalRelativeAngle(toRadians(newHeading-90) - toRadians(getHeading()))));
	}

	/**
	 * onHitRobot
	 * 
	 * need to be able to resume moving toward a corner
	 * 
	 */
	public void onHitRobot(HitRobotEvent e) {
		out.println("Oh no! I've hit a robot. ("+getX()+", "+getY()+")");
		// are we in a corner?
		if (!isInCorner) {
			// If he's in front of us, set back up a bit.
			if (e.getBearing() > -90 && e.getBearing() < 90) {
				back(100);
			} else {
				ahead(100);
			}
		}
	}
	
	public void onHitWall(HitWallEvent e) {
		double xpos = getX();
		double ypos = getY();
		double heading = getHeading();
		out.println("I've hit a wall. ("+xpos+", "+ypos+","+heading+")");

		/*if (!isInCorner) {
			if (isAtWall) {
				out.println("I'm in a corner.");
				isInCorner = true;
				
				adjustRadarSweep();
			} else {
				out.println("I've hit a wall.");
				isAtWall = true;
				
				adjustRadarSweep();
				// turn toward corner
				
				// move
				ahead(5000);
			}
		}*/
	} 

	private void adjustRadarSweep() {
		// TODO Auto-generated method stub
		
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
			
			// calculate rough linear prediction targeting
			double absoluteBearing = toRadians(getHeading()) + e.getBearingRadians();
			double adjustment = (e.getVelocity() * Math.sin(e.getHeadingRadians() - absoluteBearing) / 13.0);
			turnGunRight(toDegrees(normalRelativeAngle(absoluteBearing - toRadians(getGunHeading()) + adjustment)));
			
			// kill
			fire(bullet);
		}
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

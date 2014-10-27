package tjs.robot;

import static robocode.util.Utils.normalRelativeAngleDegrees;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.Consumer;

import robocode.BattleEndedEvent;
import robocode.BulletHitBulletEvent;
import robocode.BulletHitEvent;
import robocode.BulletMissedEvent;
import robocode.DeathEvent;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.Robot;
import robocode.RobotDeathEvent;
import robocode.RoundEndedEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.StatusEvent;
import robocode.WinEvent;

/**
 * Robot to go sit in a corner and fire towards the thick of combat
 * 
 * Based on Corners by Mathew A. Nelson and Flemming N. Larsen
 * 
 * @author Ted Snider
 *
 */
public class Sulker extends Robot {
	double[][] corners;	//  Set corners array {X,Y, left corner, right corner}
						//  array coordinates offset for width of bot
	double[] targetCorner = {18.0, 18.0, 1.0, 3.0};	// currently operation corner
	double[] sweep = {180.0, -180.0};	// Sets boundaries of radar sweep	
	int others; // Number of other robots in the game
	Map<String, Number> enemy;
	Map<String, List<Map<String, Number>>> history = new HashMap<String, List<Map<String, Number>>>();

	// Since I'm always turning something to match a given bearing and all that varies is what...
	Consumer<Double> gun = bearing -> { 
		turnGunRight(normalRelativeAngleDegrees(bearing - getGunHeading())) ;
	};
	Consumer<Double> body = bearing -> { 
		turnRight(normalRelativeAngleDegrees(bearing - getHeading())) ;
	};
	Consumer<Double> radar = bearing -> {
		turnRadarRight(normalRelativeAngleDegrees(bearing - getRadarHeading())) ;
	};

	int moveToggle = 1; // allow to flip back and forth
	// list of movement strategies utilized by move() function
	List<Consumer<Point2D.Double>> movementStrategies = Arrays.asList(
		// 0
		p -> {
			double x = getX();
			double y = getY();
			if ((x < 19 || x > 980) && (y < 19 || y > 980)) { 
				double[] corner = targetCorner;	
				double[] course = plotNextCorner(x, y); // side-effect changes current corner
				targetCorner = corner; // reset current corner
				course[1] = 100.0;
				layCourse(course);
				turnRight(135);
				moveState = 4;
			} else {
				moveState = 1;
			}
			sweep = adjustRadarSweep(x, y);
		},	
		// 1
		p -> { 
			layCourse(plotNearestCorner(p)); 
			sweep = adjustRadarSweep(getX(), getY());
			moveState = 0;
		},
		// 2
		p -> { 
			layCourse(plotNextCorner(p));  
			sweep = adjustRadarSweep(getX(), getY());
			moveState = 0;
		},
		// 3
		p -> {
			if (Math.random() > 0.5) {
				layCourse(plotNextCorner(p));
			} else {
				layCourse(plotNearestCorner(p));
			}
			sweep = adjustRadarSweep(getX(), getY());
			moveState = 0;
		},
		// 4
		p -> { 
			ahead(142); 
			sweep = adjustRadarSweep(getX(), getY());
			moveState = 5; 
		},
		// 5
		p -> { 
			back(142); 
			sweep = adjustRadarSweep(getX(), getY());
			moveState = 4; 
		},
		// 6 -- duelling state
		p -> {
			if (enemy != null) {
				double bearing = enemy.get("originalBearing").doubleValue();
				if (enemy.get("distance").doubleValue() > 200) {
					turnRight(bearing + 90 - ( 15 * moveToggle));
				} else {
					turnRight(bearing + 90);
				}
				ahead(150 * moveToggle);
				sweep[0] = bearing + 90;
				sweep[1] = bearing - 90;
			}
		}
	);
	// current movement strategy
	int moveState = 1;
	private boolean found;
	// list of radar sweep strategies
	List<Consumer<Point2D.Double>> radarStrategies = Arrays.asList(
			p -> { found = false; while (!found ) {turnRadarRight(45);} },
			p -> {
				turnToHeading(toDegrees(Math.atan2(corners[(int)targetCorner[2]][0]-p.getX(), 
						   						   corners[(int)targetCorner[2]][1]-p.getY())), radar);
				radarState = 2;
			},
			p -> {
				turnToHeading(toDegrees(Math.atan2(corners[(int)targetCorner[3]][0]-p.getX(), 
						   						   corners[(int)targetCorner[3]][1]-p.getY())), radar);
				radarState = 1;
			},
			p -> {
				turnToHeading(enemy.get("originalBearing").doubleValue()+90, radar);
				radarState = 4;
			},
			p -> {
				turnToHeading(enemy.get("originalBearing").doubleValue()-90, radar);
				radarState = 3;
			}
	);
	// current radar strategy
	int radarState = 1;
	// list of targeting strategies
	List<Consumer<Map<String, Number>>> targetingStrategies = Arrays.asList(
			// weighted linear prediction
			e -> { linearPrediction(e); },
			e -> { gaussianDistributionPrediction(e); }
	);
	
	Map<String, Map<String, Number>> scanned;	// contains vital statistics of potential targets
	Map<String, Number> identity;	// identity map for parallel stream reduce methods
	
	Map<String, Integer> stats;
	static List<Map<String, Integer>> statHistory = new ArrayList<Map<String, Integer>>();
	
	public void run() {
		double[][] setCorners = {{18.0, 18.0, 1.0, 3.0},
								 {18.0, getBattleFieldHeight()-18.0, 2.0, 0.0},
								 {getBattleFieldWidth()-18.0, getBattleFieldHeight()-18.0, 3.0, 1.0},
								 {getBattleFieldWidth()-18.0, 18.0, 0.0, 2.0}};
		corners = setCorners;
		
		// 	Set colors
		setBodyColor(Color.black);
		setGunColor(Color.white);
		setRadarColor(Color.red);
		setBulletColor(Color.red);
		setScanColor(Color.white);
		
		// Initialize statistics
		stats = new HashMap<String, Integer>(6);
		stats.put("shots", 0);
		stats.put("hits", 0);
		stats.put("misses", 0);
		stats.put("bullets", 0);
		stats.put("written", 0);
		stats.put("round", getRoundNum());

		// Save # of other bots
		others = getOthers();
		// initialize map of potential targets
		scanned = new HashMap<String, Map<String, Number>>(others);
		// initialize "blank" identity map
		identity = new HashMap<String, Number>(1);
		identity.put("score", new Double(-50000.0));

		// uncouple the gun from the body
		setAdjustGunForRobotTurn(true);
		// uncouple radar from gun
		setAdjustRadarForGunTurn(true);
		
		while (true) {
			double x = getX();
			double y = getY();
			sweep(x,y);
			shoot();
			move(x,y);
		}
	}
	
	/**
	 * onStatus: called every turn to provide a snapshot of data to the robot
	 */
	public void onStatus(StatusEvent e) {
	}
	
	private void shoot() {
		//out.println("Shooting. Strategy: "+targetingState);
		smartFire();
	}

	private void sweep(double x, double y) {
		//out.println("Sweeping. Strategy: "+radarState);
		radarStrategies.get(radarState).accept(new Point2D.Double(x, y));
	}

	private void move(double x, double y) {
		if (others == 1) {	// Begin the duel!
			moveState = 6;
			radarState = 0;
		}
		//out.println("Moving. Strategy: "+moveState);
		movementStrategies.get(moveState).accept(new Point2D.Double(x, y));
	}

	/**
	 * onHitRobot
	 * 
	 * need to be able to resume moving toward a corner
	 * 
	 */
	public void onHitRobot(HitRobotEvent e) {
		//out.println("Oh no! I've hit a robot. ("+getX()+", "+getY()+")");
		// If he's in front of us, set back up a bit.
		double bearing = e.getBearing();
		if (bearing > -90 && bearing < 90) {
			// target and fire
			if (getGunHeat() == 0 && Math.abs(bearing) < 30) {
				fire(3);
				stats.put("shots", stats.get("shots")+1);
			}
			
			back(100);
		} else {
			// target and fire
			if (getGunHeat() == 0 && Math.abs(bearing) < 30) {
				fire(3);
				stats.put("shots", stats.get("shots")+1);
			}
			
			ahead(100);
		}

		if (moveState != 6) { moveState = 3; }
	}
	
	public void onHitWall(HitWallEvent e) {
		//out.println("I've hit a wall.");
		double x = getX();
		if ((moveState == 4 || moveState== 5) && (x < 100.0 || x > 900.0)) { 
			moveState = 2; 
		} else if (moveState != 6) {
			moveState = 2;
		} else {
			moveToggle = 0 - moveToggle;
		}
	} 
	
	/**
	 * onRobotDeath: update other robot count variable for strategic adjustments
	 */
	public void onRobotDeath(RobotDeathEvent e) {
		others = getOthers();
		scanned.remove(e.getName());
		if (others == 1) {	// Begin the duel!
			moveState = 6;
			radarState = 0;
		}
		out.println(e.getName()+" died.  "+others+" remain.");
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
		//out.println("Scanned a robot.");
		found = true;
		enemy = new HashMap<String, Number>();
		enemy.put("time", getTime());
		enemy.put("distance", e.getDistance());
		enemy.put("originalBearing", e.getBearing());
		enemy.put("absoluteBearingRadians", toRadians(getHeading()) + e.getBearingRadians());
		enemy.put("velocity", e.getVelocity());
		enemy.put("heading", e.getHeading());
		enemy.put("headingRadians", e.getHeadingRadians());
		enemy.put("energy", e.getEnergy());
		enemy.put("origX", getX() +  Math.sin(enemy.get("originalBearing").doubleValue()) * enemy.get("distance").doubleValue());
		enemy.put("origY", getY() +  Math.cos(enemy.get("originalBearing").doubleValue()) * enemy.get("distance").doubleValue());
		// lower power moves faster and easier to reach distant targets
		double bullet = 1.0;
		double energy = getEnergy();
		if (enemy.get("distance").doubleValue() > 200 || energy < 15) {
			bullet = 1.0;
		} else if (enemy.get("distance").doubleValue() > 50) {
			bullet = 2 < energy ? 2 : energy;
		} else {
			bullet = 3 < energy ? 3 : energy;
		}
		enemy.put("power", bullet);
		enemy.put("wallcrawling", enemy.get("velocity").doubleValue() != 0.0 && (enemy.get("heading").doubleValue() == 0.0 || 
				enemy.get("heading").doubleValue() == 90.0 || enemy.get("heading").doubleValue() == 180.0 ||
				enemy.get("heading").doubleValue() == 270.0) ? 1 : 0);
		// add scan to list of scanned robots
		String name = e.getName();
		scanned.put(name, enemy);

		// add scan to history of that enemy's scans
		if (!(history.containsKey(name))) {
			List<Map<String, Number>> empty = new LinkedList<Map<String, Number>>(); 
			history.put(name, empty);
		} else {
			Map<String, Number> lastScan = history.get(name).get(history.get(name).size()-1);
			if (enemy.get("energy").doubleValue() != lastScan.get("energy").doubleValue()) {
				moveToggle = 0 - moveToggle;
			}
			enemy.put("prevVelo", lastScan.get("velocity").doubleValue());
			enemy.put("prevX", lastScan.get("origX").doubleValue());
			enemy.put("prevY", lastScan.get("origY").doubleValue());
			enemy.put("prevHeading", lastScan.get("heading").doubleValue());
		}
		history.get(name).add(enemy);

		// analyze history for wall-crawling behavior -- easier to hit with simple linear prediction
		long amtcrawling = history.get(name).stream()
				.filter(r -> r.get("wallcrawling").intValue() == 1)
				.count();
		double percentage = amtcrawling * 100 / history.get(name).size();
		boolean wallcrawler = percentage > 70;
		if (wallcrawler) {
			targetingStrategies.get(0).accept(enemy);
		} else {
			targetingStrategies.get(1).accept(enemy);
		}
	}
	
	public void onBulletHit(BulletHitEvent e) {
		stats.put("hits", stats.get("hits")+1);
	}
	
	public void onBulletMissed(BulletMissedEvent e) {
		stats.put("misses", stats.get("misses")+1);
	}
	
	public void onBulletHitBullet(BulletHitBulletEvent e) {
		stats.put("bullets", stats.get("bullets")+1);
	}
	
	public void onDeath(DeathEvent e) {
		if (stats.get("shots") != 0 && stats.get("written").intValue() == 0) {
			out.println("Statistics -- "+stats);
			out.println("Accuracy: "+stats.get("hits")*100/stats.get("shots"));
			stats.put("written", 1);
			statHistory.add(stats);

			int hitstat = statHistory.stream().mapToInt(s -> s.get("hits").intValue()).sum();
			int shotstat = statHistory.stream().mapToInt(s -> s.get("shots").intValue()).sum();
			out.println("Battle Shots Fired: "+shotstat);
			out.println("Battle Hits Scored: "+hitstat);
			out.println("Battle Accurracy: "+hitstat*100/shotstat);
		}
	}
	
	public void onRoundEnded(RoundEndedEvent e) {
		if (stats.get("shots") != 0 && stats.get("written").intValue() == 0) {
			out.println("Statistics -- "+stats);
			out.println("Accuracy: "+stats.get("hits")*100/stats.get("shots"));
			stats.put("written", 1);
			statHistory.add(stats);

			int hitstat = statHistory.stream().mapToInt(s -> s.get("hits").intValue()).sum();
			int shotstat = statHistory.stream().mapToInt(s -> s.get("shots").intValue()).sum();
			out.println("Battle Shots Fired: "+shotstat);
			out.println("Battle Hits Scored: "+hitstat);
			out.println("Battle Accurracy: "+hitstat*100/shotstat);
		}
	}
	
	public void onBattleEnded(BattleEndedEvent e) {
	}
	
	/**
	 * Stolen directly from TrackFire bot
	 * 
	 */
	public void onWin(WinEvent e) {
		// Victory dance
		turnRight(36000);
	}

	public void onPaint(Graphics2D g) {
		double x = getX();
		double y = getY();
		//out.println("Painting.");
		// point to the corners
		/*if (getVelocity() != 0) {
			g.setColor(Color.white);
			g.drawLine((int)x, (int)y, (int)corners[0][0], (int)corners[0][1]);
			g.drawLine((int)x, (int)y, (int)corners[1][0], (int)corners[1][1]);
			g.drawLine((int)x, (int)y, (int)corners[2][0], (int)corners[2][1]);
			g.drawLine((int)x, (int)y, (int)corners[3][0], (int)corners[3][1]);
		}*/

		// 
		g.setColor(Color.green);
		scanned.entrySet().stream()
			.filter(r -> (getTime() - r.getValue().get("time").longValue()) < 5)
			.forEach(
					r -> g.drawLine((int)x, 
								    (int)y, 
							        ((Integer)r.getValue().get("x")).intValue(), 
							        ((Integer)r.getValue().get("y")).intValue()));
	}
	
	/**
	 * linearPrediction: uses weighted linear prediction to predict movement and weight targets
	 * 
	 * @param e - ScannedRobotEvent
	 */
	private void linearPrediction(Map<String, Number> e) {
		double x = getX();
		double y = getY();
		double robotDistance = e.get("distance").doubleValue();
		double robotVelocity = e.get("velocity").doubleValue();
		double bullet = e.get("power").doubleValue();
		double absoluteBearing = e.get("absoluteBearingRadians").doubleValue();
		
		// calculate rough linear prediction targeting
		double adjustment = Math.asin((robotVelocity * Math.sin(e.get("headingRadians").doubleValue() - absoluteBearing) / Rules.getBulletSpeed(bullet)));
		// check for walls
		Point2D targPoint = checkWallCollision(new Point2D.Double(x,y), absoluteBearing + adjustment, robotDistance);
		double angle = toDegrees(Math.atan2(targPoint.getX()-x, targPoint.getY()-y));
		enemy.put("bearing", angle);
		enemy.put("adjustment", adjustment);
		enemy.put("x", new Integer((int)targPoint.getX()));
		enemy.put("y", new Integer((int)targPoint.getY()));
		
		if (enemy.get("energy").doubleValue() == 0.0) {
			enemy.put("score", 1000.0);
		} else {
			enemy.put("score",  -(adjustment + angle/30 + robotDistance));
		}
		//out.println("Added scanned robot: "+scan);
	}
	
	/**
	 * gaussianDistributionPrediction: linear prediction as above, but adjusted to factor in
	 * 		random guessing with a gaussian distribution of probability
	 * @param e
	 */
	private void gaussianDistributionPrediction(Map<String, Number> e) {
		double x = getX();
		double y = getY();
		double robotDistance = e.get("distance").doubleValue();
		double robotVelocity = 8; // e.get("velocity").doubleValue();
		double angle = 0;
		double adjustment = 0;
		Point2D targPoint = new Point2D.Double(e.get("origX").doubleValue(), e.get("origY").doubleValue());
		
		if (e.get("energy").doubleValue() != 0.0) {
			double bullet = e.get("power").doubleValue();
			double absoluteBearing =  e.get("absoluteBearingRadians").doubleValue();
			Random guess = new Random(Instant.now().toEpochMilli());
			// nextGaussian gives results -3 to 3 (three standard deviations) making 0 most likely result,
			// but robot is fairly unlikely to stand still.  So, divide by 3 to get results from -1 to 1,
			// offset by, oh, .25 (assumption: robot is going to move, but less that maximum possible),
			// then flip positive/negative at random to model forward and backward @ relative equality
			double secondGuess = ((guess.nextGaussian() / 3) + 0.25) * (guess.nextBoolean() ? 1 : -1);
			
			// calculate rough linear prediction targeting
			adjustment = Math.asin((robotVelocity * Math.sin(e.get("headingRadians").doubleValue() - absoluteBearing) / Rules.getBulletSpeed(bullet)));
			if (Double.isNaN(adjustment)) {
				out.println("Velocity "+robotVelocity);
				out.println("Heading(radians) "+e.get("headingRadians"));
				out.println("Absolute Bearing: "+absoluteBearing);
				out.println("Time to Target: "+Rules.getBulletSpeed(bullet));
				out.println("sin(heading-absolutebearing): "+Math.sin(e.get("headingRadians").doubleValue() - absoluteBearing));
				out.println("All together: "+(robotVelocity * Math.sin(e.get("headingRadians").doubleValue() - absoluteBearing) / Rules.getBulletSpeed(bullet)));
			}
			adjustment = adjustment * secondGuess;
			// check for walls
			targPoint = checkWallCollision(new Point2D.Double(x,y), absoluteBearing + adjustment, robotDistance);
		}
		angle = toDegrees(Math.atan2(targPoint.getX()-x, targPoint.getY()-y));
		enemy.put("bearing", angle);
		enemy.put("adjustment", adjustment);
		enemy.put("x", new Integer((int)targPoint.getX()));
		enemy.put("y", new Integer((int)targPoint.getY()));
		
		
		if (enemy.get("energy").doubleValue() == 0.0) {
			enemy.put("score", 1000.0);
		} else {
			enemy.put("score",  -(adjustment + angle/30 + robotDistance));
		}
	}
	
	/**
	 * smartFire: score targets, determine best target, calculate how much gun, and fire.
	 * 
	 */
	private void smartFire() {
		// abort if heat is > 0, no energy or no scanned robots yet
		if (getEnergy() > 0 && getGunHeat() == 0 && scanned.size() > 0) {
			List<Map<String, Number>> victims = new ArrayList<Map<String, Number>>(scanned.keySet().size());
			scanned.entrySet().stream()
				.filter(r -> (getTime() - r.getValue().get("time").longValue()) < 10)
				.forEach(r -> victims.add(r.getValue()));
			if (victims.size() > 0) {
				Map<String, Number> victim = victims.stream()
					.reduce(identity, (a, b) -> (a.get("score").doubleValue() > b.get("score").doubleValue()) ? a : b);
				
				//out.println("plotted bearing: "+victim.get("bearing")+", range: "+victim.get("distance"));
				turnToHeading(victim.get("bearing").doubleValue(), gun);
				//out.println("Turning gun to heading: "+gunHeading);
				
				// kill
				fire(victim.get("power").doubleValue());
				stats.put("shots", stats.get("shots")+1);
				//out.println("Firing: "+victim);
			} else {
				if (moveState != 6) { moveState = 2; }
			}
		}
	}
	
	private Point2D checkWallCollision(Point2D.Double origin, double bearing, double distance) {
		double targx = origin.getX() + Math.sin(bearing) * distance;
		if (targx < 18.0) { 
			targx = 18.0; 
		} else if (targx > (getBattleFieldWidth() - 18.0)) {
			targx = getBattleFieldWidth() - 18.0;
		}
		double targy = origin.getY() + Math.cos(bearing) * distance;
		if (targy < 18.0) { 
			targy = 18.0; 
		} else if (targy > (getBattleFieldHeight() - 18.0)) {
			targy = getBattleFieldHeight() - 18.0;
		}
		return new Point2D.Double(targx, targy);
	}
	
	/**
	 * layCourse: combination method to turn to a heading and move a distance
	 * 
	 * @param destination - double[] containing heading and distance
	 */
	private void layCourse(double[] destination) {
		turnToHeading(destination[0], body);
		ahead(destination[1]);
	}

	private double[] plotNearestCorner(Point2D.Double p) {
			return plotNearestCorner(p.getX(), p.getY());
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
		return plotCorner(targetCorner[0]-x, targetCorner[1]-y);
	}

	private double[] plotNextCorner(Point2D.Double p) {
		return plotNextCorner(p.getX(), p.getY());
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
		targetCorner = corners[(int)targetCorner[2]];
		return plotCorner(targetCorner[0]-x, targetCorner[1]-y);
	}
	
	private double[] plotCorner(double x, double y) {
		double[] plot = {toDegrees(Math.atan2(x, y)), Math.sqrt(x*x + y*y)};
		return plot;
	}

	private double[] adjustRadarSweep(Point2D.Double p) {
		return adjustRadarSweep(p.getX(), p.getY());
	}
	
	private double[] adjustRadarSweep(double x, double y) {
		double [] bearings = {toDegrees(Math.atan2(corners[(int)targetCorner[2]][0]-x, 
												   corners[(int)targetCorner[2]][1]-y)), 
							  toDegrees(Math.atan2(corners[(int)targetCorner[3]][0]-x, 
									  			   corners[(int)targetCorner[3]][1]-y))};
		return bearings;
	}
	
	/**
	 * Constantly turning things right to match new bearings
	 * @param theta - bearing to match
	 * @param operator - body part (body, gun, radar) to turn
	 */
	private void turnToHeading(double theta, Consumer<Double> operator) {
		operator.accept(theta);
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

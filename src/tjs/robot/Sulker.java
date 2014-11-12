package tjs.robot;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static robocode.util.Utils.normalAbsoluteAngle;
import static robocode.util.Utils.normalRelativeAngle;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.RoundRectangle2D;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import robocode.AdvancedRobot;
import robocode.BattleEndedEvent;
import robocode.BulletHitBulletEvent;
import robocode.BulletHitEvent;
import robocode.BulletMissedEvent;
import robocode.Condition;
import robocode.CustomEvent;
import robocode.DeathEvent;
import robocode.HitRobotEvent;
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
public class Sulker extends AdvancedRobot {
	// movement and battlefield dimension attributes.
	Point2D destination;
	Rectangle2D boundries;
	double margin = 40;
	double minX = margin;
	double minY = margin;
	double rectHeight;
	double rectWidth;
	int others; // Number of other robots in the game
	int moveToggle = 1; // allow movement direction to flip back and forth
	int radarToggle = 1; // allow radar to flip back and forth

	// Maps for handling scanning of enemy bots
	Map<Integer, Map<String, Number>> scanned; // contains vital statistics of
												// potential targets
	Map<String, Number> enemy; // current working potential target
	Map<Integer, List<Map<String, Number>>> history = new HashMap<Integer, List<Map<String, Number>>>();
								// history of scanned targets
	Map<String, Number> identity; // identity map for parallel stream reduce methods
	// number of virtual guns
	int gunArraySize = 65;

	Map<String, Integer> stats;
	static List<Map<String, Integer>> statHistory = new ArrayList<Map<String, Integer>>();

	// Consumers for turning various tank parts to an absolute heading
	Consumer<Double> gun = bearing -> {
		setTurnGunRightRadians(normalRelativeAngle(bearing - getGunHeadingRadians()));
	};
	Consumer<Double> body = bearing -> {
		setTurnRightRadians(normalRelativeAngle(bearing - getHeadingRadians()));
	};
	Consumer<Double> radar = bearing -> {
		setTurnRadarRightRadians(normalRelativeAngle(bearing - getRadarHeadingRadians()));
	};

	// list of movement states utilized by move() function
	List<Consumer<Point2D>> movementStates;
	// current movement strategy
	int moveState = initializeMovementStates();
	
	// list of radar sweep strategies
	List<Consumer<Point2D>> radarStrategies = Arrays.asList(
			p -> {
				turnRadarRight(Double.POSITIVE_INFINITY);
			},
			p -> {
				Point2D next = plotNextCorner(p);
				turnToHeading(atan2(next.getX() - p.getX(), next.getY() - p.getY()), radar);
				radarState = 2;
			},
			p -> {
				Point2D prev = plotPreviousCorner(p);
				turnToHeading(atan2(prev.getX() - p.getX(), prev.getY() - p.getY()), radar);
				radarState = 1;
			},
			p -> {
				turnToHeading(enemy.get("originalBearing").doubleValue() + PI/2, radar);
				radarState = 4;
			},
			p -> {
				turnToHeading(enemy.get("originalBearing").doubleValue() - PI/2, radar);
				radarState = 3;
			});
	// current radar strategy
	int radarState = 1;
	
	// list of targeting strategies
	List<Consumer<Map<String, Number>>> targetingStrategies = Arrays.asList(
			// weighted linear prediction
			e -> {
				linearPrediction(e);
			}, 
			// gaussian-distribution-modified, weighted linear prediction
			e -> {
				gaussianDistributionPrediction(e);
			},
			e -> {
				guessFactorPrediction(e);
			});

	// List of options for gun ready states
	List<Consumer<Map<String, Number>>> gunActions = Arrays.asList(
			g -> {
				turnToHeading(g.get("bearing").doubleValue(), gun);
				gunState = 1;
			}, 
			g -> {
				setFire(g.get("power").doubleValue());
				g.put("fired", 1);
				stats.put("shots", stats.get("shots") + 1);
				gunState = 0;
			});
	// current ready gun state
	int gunState = 0;
	
	BiFunction<Map<String, Number>, Map<String, Number>, Integer> calculateWaveHit = (wave, scan) -> {
		Integer gunBearing = null;
		double startX = wave.get("myX").doubleValue();
		double startY = wave.get("myY").doubleValue();
		double enemyX = scan.get("origX").doubleValue();
		double enemyY = scan.get("origY").doubleValue();
		double startBearing = wave.get("absoluteBearingRadians").doubleValue();
		double maxEscapeAngle = wave.get("escapeAngle").doubleValue();
		int direction = wave.get("direction").intValue();
		if (wave.get("factor") == null) {
			double waveDistance = Point2D.distance(startX, startY, enemyX, enemyY);
			if (waveDistance <= 
					(scan.get("time").doubleValue() - wave.get("time").doubleValue()) 
					* wave.get("bulletSpeed").doubleValue()) {
				double desiredDirection = atan2(enemyX - startX, enemyY - startY);
				double angleOffset = normalRelativeAngle(desiredDirection - startBearing);
				double guessFactor = Math.max(-1, Math.min(1,  angleOffset / maxEscapeAngle)) * direction;
				gunBearing = (int)Math.round((gunArraySize - 1) / 2 * (guessFactor + 1));
			}
		} else {
			gunBearing = wave.get("factor").intValue();
		}
		
		return gunBearing;
	};
	
	public void run() {
		initialize();

		double x = getX();
		double y = getY();

		sweep(new Point2D.Double(x, y));
		move(new Point2D.Double(x, y));

		while (true) {
			execute();
		}
	}

	/*
	private void shoot() {
		// out.println("Shooting. Strategy: "+targetingState);
		smartFire();
	}
	*/

	private void sweep(Point2D p) {
		// out.println("Sweeping. Strategy: "+radarState);
		radarStrategies.get(radarState).accept(p);
	}

	private void move(Point2D p) {
		if (others == 1) { // Begin the duel!
			moveState = 5;
			radarState = 0;
		}
		// out.println("Moving. Strategy: "+moveState);
		movementStates.get(moveState).accept(p);
	}
	
	/**
	 * onCustomEvent
	 * 
	 * handle user-defined events.
	 * 
	 * Currently when radar completes a sweep or when movement needs rechecked
	 * 
	 */
	public void onCustomEvent(CustomEvent e) {
		String cond = e.getCondition().getName();
		
		switch (cond) {
			case "sweepComplete" : 
				if (radarState != 0) {
					sweep(new Point2D.Double(getX(), getY()));
				}
				//shoot();
				break;
			case "moveComplete" :
				if (moveState != 5) {
					move(new Point2D.Double(getX(), getY()));
				}
				break;
			case "turnComplete" :
				setMaxVelocity(Rules.MAX_VELOCITY);
				if (moveState != 5) {
					move(new Point2D.Double(getX(), getY()));
				}
				break;
		}
	}
	
	/**
	 * onStatus
	 * 
	 * fires every turn
	 * 
	 */
	public void onStatus(StatusEvent e) {
		if (boundries != null && moveState != 5) {
			double heading = getHeadingRadians();
			double velocity = Rules.MAX_VELOCITY;
			double xo = getX();
			double yo = getY();
			double xd = xo + Math.sin(heading) * velocity;
			double yd = yo + Math.cos(heading) * velocity; 
			if (!boundries.contains(xo, yo)) {
				recenter(new Point2D.Double(xo, yo));
			} else if (getTurnRemainingRadians() != 0) {
				while (!boundries.contains(xd, yd) && velocity >= 0) {
					velocity--;
					xd = xo + Math.sin(heading) * velocity;
					yd = yo + Math.cos(heading) * velocity; 
				}
				if (velocity < 0) {
					recenter(new Point2D.Double(xo, yo));
				}
			} 
			setMaxVelocity(velocity);
		}
	}

	/**
	 * onHitRobot
	 * 
	 * need to be able to resume moving toward a corner
	 * 
	 */
	public void onHitRobot(HitRobotEvent e) {
		recenter(new Point2D.Double(getX(), getY()));
	}

	/**
	 * onRobotDeath: update other robot count variable for strategic adjustments
	 */
	public void onRobotDeath(RobotDeathEvent e) {
		others = getOthers();
		scanned.remove(e.getName());
		if (others == 1) { // Begin the duel!
			moveState = 5;
			radarState = 0;
		}
		out.println(e.getName() + " died.  " + others + " remain.");
	}

	/**
	 * onScannedRobot
	 * 
	 * 1. check heat 2. calculate shot power based in distance...weaker/faster
	 * for further targets 3. determine an amount to lead target based on
	 * current velocity
	 * 
	 * @param ScannedRobotEvent
	 *            e - event pertaining to robot scanned
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		// out.println("Scanned a robot.");
		double time = getTime();
		double distance = e.getDistance();
		double bearing = e.getBearingRadians();
		double myHeading = getHeadingRadians();
		double heading = e.getHeadingRadians();
		double absoluteBearing = bearing + myHeading;
		double velocity = e.getVelocity();
		double myEnergy = getEnergy();
		double energy = e.getEnergy();
		double myX = getX();
		double myY = getY();
		double origX = myX + Math.sin(absoluteBearing) * distance;
		double origY = myY + Math.cos(absoluteBearing) * distance;
		double bullet = 1.0;
		if (distance > 400 ||  myEnergy < 15) {
			bullet = 1.0;
		} else if (distance > 200) {
			bullet = 2 < myEnergy ? 2 : myEnergy;
		} else {
			bullet = 3 < myEnergy ? 3 : myEnergy;
		}
		int name = e.getName().hashCode();
		double bulletSpeed = Rules.getBulletSpeed(bullet);
		double escapeAngle = Math.asin(Rules.MAX_VELOCITY / bulletSpeed);
		int direction = Math.sin(heading - absoluteBearing) * velocity < 0 ? -1 : 1;
		Integer factor = null; 
		int distanceSeg = (int)(Math.floor(distance / 100) / bulletSpeed);
		int bearingSeg = (int)Math.floor(Math.abs(bearing) / (PI/6));
		int wWallSeg = Math.min(8, (int)Math.floor(origX/36));
		int eWallSeg = Math.min(8, (int)Math.floor((boundries.getMaxX()+margin-origX)/36));
		int sWallSeg = Math.min(8, (int)Math.floor(origY/36));
		int nWallSeg = Math.min(8, (int)Math.floor((boundries.getMaxY()+margin-origY)/36));

		enemy = new HashMap<String, Number>();
		enemy.put("time", time);
		enemy.put("distance", distance);
		enemy.put("originalBearing", bearing);
		enemy.put("absoluteBearingRadians", absoluteBearing);
		enemy.put("velocity", velocity);
		enemy.put("heading", heading);
		enemy.put("headingRadians", heading);
		enemy.put("myHeading", myHeading);
		enemy.put("energy", energy);
		enemy.put("myX", myX);
		enemy.put("myY", myY);
		enemy.put("origX", origX);
		enemy.put("origY", origY);
		enemy.put("power", bullet);
		enemy.put("name", name);
		enemy.put("wallcrawling",
				velocity != 0 && (heading == 0 || heading == PI/2 || heading == PI || heading == 1.5 * PI) ? 1 : 0);
		enemy.put("bulletSpeed", bulletSpeed);
		enemy.put("escapeAngle", escapeAngle);
		enemy.put("direction", direction);
		enemy.put("factor", factor);
		enemy.put("distanceSeg", distanceSeg);
		enemy.put("bearingSeg", bearingSeg);
		enemy.put("wWallSeg", wWallSeg);
		enemy.put("eWallSeg", eWallSeg);
		enemy.put("sWallSeg", sWallSeg);
		enemy.put("nWallSeg", nWallSeg);
		enemy.put("fired", 0);
		
		// add scan to list of scanned robots
		scanned.put(name, enemy);

		// add scan to history of that enemy's scans
		if (!(history.containsKey(name))) {
			List<Map<String, Number>> empty = new ArrayList<Map<String, Number>>();
			history.put(name, empty);
		} else {
			List<Map<String, Number>> namedHistory =  history.get(name);
			
			namedHistory.stream().filter(m -> m.get("factor") == null)
				.forEach(m -> { m.put("factor", calculateWaveHit.apply(m, enemy)); });
		}
		history.get(name).add(enemy);

		// analyze history for wall-crawling behavior -- easier to hit with
		// simple linear prediction
		long amtcrawling = history.get(name).stream()
				.filter(r -> r.get("wallcrawling").intValue() == 1).count();
		double percentage = amtcrawling * 100 / history.get(name).size();
		boolean wallcrawler = percentage > 70;
		if (wallcrawler) {
			targetingStrategies.get(0).accept(enemy);
		}  else if (others > 4) {
			targetingStrategies.get(1).accept(enemy);
		} else {
			targetingStrategies.get(2).accept(enemy);
		}
		smartFire();
		if (moveState == 5) {
			move(new Point2D.Double(getX(), getY()));
		}
		if (radarState == 0) {
			setTurnRadarRightRadians(normalRelativeAngle(absoluteBearing - getRadarHeadingRadians()) * 2);
		}
		//out.println("Scanned robot: "+enemy);
	}

	public void onBulletHit(BulletHitEvent e) {
		stats.put("hits", stats.get("hits") + 1);
	}

	public void onBulletMissed(BulletMissedEvent e) {
		stats.put("misses", stats.get("misses") + 1);
	}

	public void onBulletHitBullet(BulletHitBulletEvent e) {
		stats.put("bullets", stats.get("bullets") + 1);
	}

	public void onDeath(DeathEvent e) {
		if (stats.get("shots") != 0 && stats.get("written").intValue() == 0) {
			out.println("Statistics -- " + stats);
			out.println("Accuracy: " + stats.get("hits") * 100
					/ stats.get("shots"));
			stats.put("written", 1);
			statHistory.add(stats);

			int hitstat = statHistory.stream()
					.mapToInt(s -> s.get("hits").intValue()).sum();
			int shotstat = statHistory.stream()
					.mapToInt(s -> s.get("shots").intValue()).sum();
			out.println("Battle Shots Fired: " + shotstat);
			out.println("Battle Hits Scored: " + hitstat);
			out.println("Battle Accurracy: " + hitstat * 100 / shotstat);
		}
	}

	public void onRoundEnded(RoundEndedEvent e) {
		if (stats.get("shots") != 0 && stats.get("written").intValue() == 0) {
			out.println("Statistics -- " + stats);
			out.println("Accuracy: " + (double)stats.get("hits") * 100
					/ (double)stats.get("shots"));
			stats.put("written", 1);
			statHistory.add(stats);
			
			out.println("Number of rounds: "+getTime());
			
			int hitstat = statHistory.stream()
					.mapToInt(s -> s.get("hits").intValue()).sum();
			int shotstat = statHistory.stream()
					.mapToInt(s -> s.get("shots").intValue()).sum();
			out.println("Battle Shots Fired: " + shotstat);
			out.println("Battle Hits Scored: " + hitstat);
			out.println("Battle Accurracy: " + (double)hitstat * 100 / (double)shotstat);
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
		double heading = getHeadingRadians();
		double headlightX = x + Math.sin(heading) * 100;
		double headlightY = y + Math.cos(heading) * 100;
		g.setColor(Color.white);
		g.drawLine((int)x, (int)y, (int)headlightX, (int)headlightY);
		
		g.setColor(Color.yellow);
		g.drawLine((int)x, (int)y, (int)destination.getX(), (int)destination.getY());

		RoundRectangle2D corner1;
		RoundRectangle2D corner2;
		RoundRectangle2D corner3;
		RoundRectangle2D corner4;
		// dodging about in a corner has a looser definition of corner than
		// other movement states
			corner1 = new RoundRectangle2D.Double(0, 0, 100, 100, 36, 36);
			corner2 = new RoundRectangle2D.Double(0,
					getBattleFieldWidth() - 100, 100, 100, 36, 36);
			corner3 = new RoundRectangle2D.Double(getBattleFieldHeight() - 100,
					getBattleFieldWidth() - 100, 100, 100, 36, 36);
			corner4 = new RoundRectangle2D.Double(getBattleFieldHeight() - 100,
					0, 100, 100, 36, 36);
		g.setColor(Color.magenta);
		g.draw(corner1);
		g.draw(corner2);
		g.draw(corner3);
		g.draw(corner4);
		g.draw(boundries);

		/*
		 * g.setColor(Color.green); scanned.entrySet().stream() .filter(r ->
		 * (getTime() - r.getValue().get("time").longValue()) < 5) .forEach( r
		 * -> g.drawLine((int)x, (int)y,
		 * ((Integer)r.getValue().get("x")).intValue(),
		 * ((Integer)r.getValue().get("y")).intValue()));
		 */
	}

	private void recenter(Point2D origin) {
		moveState = 6;
		move(origin);
	}

	/**
	 * linearPrediction: uses weighted linear prediction to predict movement and
	 * weight targets
	 * 
	 * @param e
	 *            - ScannedRobotEvent
	 */
	private void linearPrediction(Map<String, Number> e) {
		double x = getX();
		double y = getY();
		double robotDistance = e.get("distance").doubleValue();
		double robotVelocity = e.get("velocity").doubleValue();
		double bullet = e.get("power").doubleValue();
		double absoluteBearing = e.get("absoluteBearingRadians").doubleValue();

		// calculate rough linear prediction targeting
		double adjustment = Math.asin((robotVelocity
				* Math.sin(e.get("headingRadians").doubleValue()
						- absoluteBearing) / Rules.getBulletSpeed(bullet)));
		// check for walls
		Point2D targPoint = checkWallCollision(new Point2D.Double(x, y),
				absoluteBearing + adjustment, robotDistance);
		double angle = atan2(targPoint.getX() - x, targPoint.getY() - y);
		enemy.put("bearing", angle);
		enemy.put("adjustment", adjustment);
		enemy.put("x", new Integer((int) targPoint.getX()));
		enemy.put("y", new Integer((int) targPoint.getY()));

		if (enemy.get("energy").doubleValue() == 0.0) {
			enemy.put("score", 1000.0);
		} else {
			enemy.put("score", -(adjustment + angle / (PI / 4) + robotDistance));
		}
		// out.println("Added scanned robot: "+scan);
	}

	/**
	 * gaussianDistributionPrediction: linear prediction as above, but adjusted
	 * to factor in random guessing with a gaussian distribution of probability
	 * 
	 * @param e
	 */
	private void gaussianDistributionPrediction(Map<String, Number> e) {
		double x = getX();
		double y = getY();
		double robotDistance = e.get("distance").doubleValue();
		double robotVelocity = 8; // e.get("velocity").doubleValue();
		double angle = 0;
		double adjustment = 0;
		Point2D targPoint = new Point2D.Double(e.get("origX").doubleValue(), e
				.get("origY").doubleValue());

		if (e.get("energy").doubleValue() != 0.0) {
			double bullet = e.get("power").doubleValue();
			double absoluteBearing = e.get("absoluteBearingRadians")
					.doubleValue();
			Random guess = new Random(Instant.now().toEpochMilli());
			// nextGaussian gives results -3 to 3 (three standard deviations)
			// making 0 most likely result,
			// but robot is fairly unlikely to stand still. So, divide by 3 to
			// get results from -1 to 1,
			// offset by, oh, .25 (assumption: robot is going to move, but less
			// that maximum possible),
			// then flip positive/negative at random to model forward and
			// backward @ relative equality
			double secondGuess = ((guess.nextGaussian() / 3) + 0.25)
					* (guess.nextBoolean() ? 1 : -1);

			// calculate rough linear prediction targeting
			adjustment = Math.asin((robotVelocity
					* Math.sin(e.get("headingRadians").doubleValue()
							- absoluteBearing) / Rules.getBulletSpeed(bullet)));
			if (Double.isNaN(adjustment)) {
				out.println("Velocity " + robotVelocity);
				out.println("Heading(radians) " + e.get("headingRadians"));
				out.println("Absolute Bearing: " + absoluteBearing);
				out.println("Time to Target: " + Rules.getBulletSpeed(bullet));
				out.println("sin(heading-absolutebearing): "
						+ Math.sin(e.get("headingRadians").doubleValue()
								- absoluteBearing));
				out.println("All together: "
						+ (robotVelocity
								* Math.sin(e.get("headingRadians")
										.doubleValue() - absoluteBearing) / Rules
									.getBulletSpeed(bullet)));
			}
			adjustment = adjustment * secondGuess;
			// check for walls
			targPoint = checkWallCollision(new Point2D.Double(x, y),
					absoluteBearing + adjustment, robotDistance);
		}
		angle = enemy.get("absoluteBearingRadians").doubleValue();
		enemy.put("bearing", angle);
		enemy.put("adjustment", adjustment);
		enemy.put("x", new Integer((int) targPoint.getX()));
		enemy.put("y", new Integer((int) targPoint.getY()));

		if (enemy.get("energy").doubleValue() == 0.0) {
			enemy.put("score", 1000.0);
		} else {
			enemy.put("score", -(adjustment / (PI / 4) + robotDistance));
		}
	}
	
	/**
	 * guessFactorPrediction: uses an adaptation of the GuessFactor targeting tutorial
	 * 
	 * @param enemy
	 */
	private void guessFactorPrediction(Map<String, Number> enemy) {
		int hitThreshold = 3;
		List<Map<String, Number>> waves = history.get(enemy.get("name"));
		Map<Double, Long> angles =  waves.stream()
				//.sorted((a, b) -> Integer.compare(a.get("time").intValue() + a.get("fired").intValue() * 10000, 
				//	   						   b.get("time").intValue() + b.get("fired").intValue() * 10000))
				.filter(w -> w.get("factor") != null &&
					 	  Math.abs(Math.min(Math.min(w.get("nWallSeg").doubleValue(), w.get("sWallSeg").doubleValue()),
					 			  			Math.min(w.get("eWallSeg").doubleValue(), w.get("wWallSeg").doubleValue())) -
					 			 Math.min(Math.min(enemy.get("nWallSeg").doubleValue(), enemy.get("sWallSeg").doubleValue()),
					 					 Math.min(enemy.get("eWallSeg").doubleValue(),
					 							 enemy.get("wWallSeg").doubleValue()))) <= 2 &&
					 	  Math.abs(w.get("distanceSeg").intValue() - enemy.get("distanceSeg").intValue()) <= 3 &&
					 	  w.get("direction").intValue() == enemy.get("direction").intValue() &&
					 	  w.get("velocity").doubleValue() == enemy.get("velocity").doubleValue())
				//.limit(gunArraySize * (hitThreshold + 1))
				.collect(Collectors.groupingBy(w -> w.get("factor").doubleValue(), 
											Collectors.counting()));
		if (angles.size() > 0) {
			double modeOffset = angles.entrySet()
									  .stream()
									  .reduce((a,b) -> a.getValue() > b.getValue() ? a : b)
									  .get()
									  .getKey();
			if (angles.get(modeOffset) > hitThreshold) {
				//out.println("Chosen angle offset "+modeOffset+" radians with count of "+angles.get(modeOffset));
				double heading = enemy.get("heading").doubleValue();
				double absoluteBearing = enemy.get("absoluteBearingRadians").doubleValue();
				double velocity = enemy.get("velocity").doubleValue();
				int direction = Math.sin(heading - absoluteBearing) * velocity < 0 ? -1 : 1;
				double escapeAngle = enemy.get("escapeAngle").doubleValue();
				double guessFactor = (double)(modeOffset - (gunArraySize - 1) / 2) / ((gunArraySize - 1) / 2);

				// adjust for my movment...assume one turn
				double myVelocity = getVelocity();
				if (myVelocity != 0) {
					double myHeading = normalAbsoluteAngle(getHeadingRadians() + getTurnRemainingRadians());
					double nextX = enemy.get("myX").doubleValue() + Math.sin(myHeading) * myVelocity;
					double nextY = enemy.get("myY").doubleValue() + Math.cos(myHeading) * myVelocity;
					double newAbsoluteBearing = atan2(enemy.get("origX").doubleValue() - nextX,
													  enemy.get("origY").doubleValue() - nextY);
					enemy.put("bearing", newAbsoluteBearing);
				} else {
					enemy.put("bearing", enemy.get("absoluteBearingRadians").doubleValue());					
				}
				enemy.put("adjustment", direction * guessFactor * escapeAngle);
				if (enemy.get("energy").doubleValue() == 0.0) {
					enemy.put("score", 1000.0);
				} else {
					enemy.put("score", -(modeOffset / (PI / 4) + enemy.get("distance").doubleValue()));
				}
			}
		}
	}

	/**
	 * smartFire: score targets, determine best target, calculate how much gun,
	 * and fire.
	 * 
	 */
	private void smartFire() {
		// abort if heat is > 0, no energy or no scanned robots yet
		if (getEnergy() > 0 && scanned.size() > 0 && getGunHeat() == 0 && getGunTurnRemaining() == 0) {
			List<Map<String, Number>> victims = new ArrayList<Map<String, Number>>(
					scanned.keySet().size());
			scanned.entrySet()
					.stream()
					.filter(r -> (getTime() - r.getValue().get("time")
							.longValue()) < 10 && r.getValue().get("score") != null)
					.forEach(r -> victims.add(r.getValue()));
			if (victims.size() > 0) {
				Map<String, Number> victim = victims.stream().reduce(
						identity,
						(a, b) -> (a.get("score").doubleValue() > b
								.get("score").doubleValue()) ? a : b);
				gunActions.get(gunState).accept(victim);
			}
		}
	}

	private Point2D checkWallCollision(Point2D.Double origin, double bearing,
			double distance) {
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
	 * inCorner: calculates whether or not current position is close enough to
	 * being in a corner.
	 * 
	 * @param p
	 *            - Point2D representation of current location
	 * @return - boolean indicating if we are in position approximately in a
	 *         corner
	 */
	private boolean inCorner(Point2D p) {
		RoundRectangle2D corner1;
		RoundRectangle2D corner2;
		RoundRectangle2D corner3;
		RoundRectangle2D corner4;
		corner1 = new RoundRectangle2D.Double(0, 0, 100, 100, 36, 36);
		corner2 = new RoundRectangle2D.Double(0,
				getBattleFieldWidth() - 100, 100, 100, 36, 36);
		corner3 = new RoundRectangle2D.Double(getBattleFieldHeight() - 100,
				getBattleFieldWidth() - 100, 100, 100, 36, 36);
		corner4 = new RoundRectangle2D.Double(getBattleFieldHeight() - 100,
				0, 100, 100, 36, 36);
		return corner1.contains(p) || corner2.contains(p)
				|| corner3.contains(p) || corner4.contains(p);
	}

	/**
	 * layCourse: combination method to turn toward and move to a point
	 * 
	 * @param origin - Point2D representation of the starting coordinates
	 * @param target - Point2D representation of the destination coordinates
	 */
	private void layCourse(Point2D origin, Point2D target) {
		double bearing = atan2(target.getX()-origin.getX(), target.getY()-origin.getY());
		turnToHeading(bearing, body);
		setAhead(origin.distance(target));
	}

	/**
	 * plotNearestCorner(Point2D): calculates nearest corner from
	 * passed-in location
	 * 
	 * @param p - current coordinates
	 * @return Point2D - coordinates of nearest corner
	 */
	private Point2D plotNearestCorner(Point2D p) {
		Point2D corner = new Point2D.Double((p.getX() < boundries.getCenterX()) ? boundries.getMinX() : boundries.getMaxX(), 
											(p.getY() < boundries.getCenterY()) ? boundries.getMinY() : boundries.getMaxY());
		return corner;
	}

	/**
	 * plotNextCorner(Point2D): calculates coordinates of next clockwise corner from
	 * passed-in location
	 * 
	 * @param p - current coordinates
	 * @return Point2D - coordinates of next clockwise corner
	 */
	private Point2D plotNextCorner(Point2D p) {
		Point2D corner = new Point2D.Double((p.getY() < boundries.getCenterY()) ? boundries.getMinX() : boundries.getMaxX(), 
											(p.getX() < boundries.getCenterX()) ? boundries.getMaxY() : boundries.getMinY());
		return corner;
	}

	/**
	 * plotPreviousCorner(Point2D): calculates direction and distance of
	 * previous corner in corners array from the passed-in location
	 * 
	 * @param p - current coordinates
	 * @return double array of polar coordinates for course (angle, distance)
	 */
	private Point2D plotPreviousCorner(Point2D p) {
		Point2D corner = new Point2D.Double((p.getY() < boundries.getCenterY()) ? boundries.getMaxX() : boundries.getMinX(), 
											(p.getX() < boundries.getCenterX()) ? boundries.getMinY() : boundries.getMaxY());
		return corner;
	}

	/**
	 * Constantly turning things right to match new bearings
	 * 
	 * @param theta
	 *            - bearing to match
	 * @param operator
	 *            - body part (body, gun, radar) to turn
	 */
	private void turnToHeading(double theta, Consumer<Double> operator) {
		operator.accept(theta);
	}

	private void initialize() {
		double battleFieldHeight = getBattleFieldHeight();
		double battleFieldWidth = getBattleFieldWidth();
		rectHeight = battleFieldHeight - (2 * margin);
		rectWidth = battleFieldWidth - (2 * margin);
		boundries = new Rectangle2D.Double(minX, minY, rectWidth, rectHeight);
		destination = new Point2D.Double(minX, minY);

		// Set colors
		setBodyColor(Color.black);
		setGunColor(Color.white);
		setRadarColor(Color.red);
		setBulletColor(Color.white);
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
		if (others == 1) { radarState = 0; }
		
		// initialize map of potential targets
		scanned = new HashMap<Integer, Map<String, Number>>(others);
		// initialize "blank" identity map
		identity = new HashMap<String, Number>(1);
		identity.put("score", new Double(-50000.0));

		// decouple the gun from the body
		setAdjustGunForRobotTurn(true);
		// decouple radar from gun
		setAdjustRadarForGunTurn(true);
		
		addCustomEvent(new Condition("moveComplete") { 
			public boolean test() { 
				return getDistanceRemaining() == 0; 
			}
		});
		
		addCustomEvent(new Condition("turnComplete") {
			public boolean test() {
				return getTurnRemainingRadians() == 0;
			}
		});
		
		addCustomEvent(new Condition("sweepComplete") {
			public boolean test() {
				return getRadarTurnRemainingRadians() == 0;
			}
		});
	}

	private int initializeMovementStates() {
		if (movementStates == null) {
			movementStates = new ArrayList<Consumer<Point2D>>();
		}

		// 0 - rebasing to nearest corner
		movementStates.add(p -> {
			if (!inCorner(p)) {
				destination.setLocation(plotNearestCorner(p));
				layCourse(p, destination);
			} else {
				moveState = 4;
			}
		});
		// 1 - rebasing to next corner
		movementStates.add(p -> {
			if (!inCorner(p)) {
				destination.setLocation(plotNextCorner(p));
				layCourse(p, destination);
			} else {
				moveState = 4;
			}
		});
		// 2 - rebasing to previous corner
		// UNUSED
		movementStates.add(p -> {
			if (!inCorner(p)) {
				destination.setLocation(plotPreviousCorner(p));
				layCourse(p, destination);
			} else {
				moveState = 4;
			}
		});
		// 3 - traveling to selected base corner
		// UNUSED
		movementStates.add(p -> {
			out.println("WOAH. IN UNUSED MOVEMENT CASE 3.");
		});
		// 4 - dodging about in the corner
		movementStates.add(p -> {
			if (inCorner(p)) {
				if (getVelocity() >= 0 || Math.random() > .1) {
					double x = boundries.getMinX() + boundries.getCenterX()
							* Math.random();
					double y = boundries.getMinY() + boundries.getCenterY()
							* Math.random();
					destination.setLocation(x, y);
					layCourse(p, destination);
				}
			} else {
				moveState = 0;
			}
		});
		// 5 - stalking last enemy
		movementStates.add(p -> {
			if (!boundries.contains(p)) {
				layCourse(p, new Point2D.Double(boundries.getCenterX(), boundries.getCenterY()));
			} else if (enemy != null && enemy.get("time").doubleValue() > getTime() - 10) {
				final double WALL_MARGIN = 100;
				Rectangle2D fieldRectangle = new Rectangle2D.Double(WALL_MARGIN, WALL_MARGIN,
						getBattleFieldWidth() - WALL_MARGIN * 2, getBattleFieldHeight() - WALL_MARGIN * 2);
				double direction = 0.4 * moveToggle;
				double enemyAbsoluteBearing = enemy.get("absoluteBearingRadians").doubleValue();
				double enemyDistance = enemy.get("distance").doubleValue();
				Point2D enemyLocation = new Point2D.Double(enemy.get("origX").doubleValue(),
														   enemy.get("origY").doubleValue());
				Point2D robotDestination;
				double tries = 0;
				double length = enemyDistance * (1.2 - tries / 100.0);
				double newX = enemyLocation.getX() + Math.sin(enemyAbsoluteBearing + PI + direction) * length;
				double newY = enemyLocation.getY() + Math.cos(enemyAbsoluteBearing + PI + direction) * length;
				while (!fieldRectangle.contains(robotDestination = new Point2D.Double(newX, newY)) && tries < 125) {
					tries++;
					length = enemyDistance * (1.2 - tries / 100.0);
					newX = enemyLocation.getX() + Math.sin(enemyAbsoluteBearing + PI + direction) * length;
					newY = enemyLocation.getY() + Math.cos(enemyAbsoluteBearing + PI + direction) * length;
				}
				if ((Math.random() < (11 / 0.421075) / enemyDistance || tries > (enemyDistance / 11 / 0.699484))) {
					moveToggle = -moveToggle;
				}
				// Jamougha's cool way
				destination = robotDestination;
				double angle = atan2(robotDestination.getX() - p.getX(), 
						robotDestination.getY() - p.getY()) - enemy.get("myHeading").doubleValue();
				setAhead(Math.cos(angle) * 100);
				setTurnRightRadians(Math.tan(angle));
			} else {
				moveState = 6;
			}
			if (radarState != 0) { radarState = 0; }
		});
		// 6 - recentering
		movementStates.add(p -> {
			if (!boundries.contains(p)) {
				layCourse(p, new Point2D.Double(boundries.getCenterX(), boundries.getCenterY()));
			} else if (others == 1) {
				moveState = 5;
			} else {
				moveState = 0;
			}
		});
		
		return 0;
	}

}

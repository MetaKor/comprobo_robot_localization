### Computational Robotics Fall 2022

# Robot Localization Project

### Luke Raus and Han Vakil


## Project Goal

The goal of this project is to solve the robot localization problem by implementing a particle filter algorithm. This task entails estimating the true position of a robot traversing a known/mapped space given data from its onboard sensors (in our case, its odometry and laser scanner/LiDAR sensor), a rough initial guess at its position, and a complete map of the space through which it navigates. Accurate localization in this manner is pivotal to many robotic applications in relatively structured spaces, such as offices or warehouses.

We successfully implemented the particle filter, using many ideas from Thrun et al.'s *Probabilistic Robotics*, and used it to localize a NEATO robot in a simulated "gauntlet world."


## Demonstration

![Particle filter demo in Gauntlet world](/media/gauntlet_demo.gif)

This video shows our particle filter algorithm successfully converge on the actual pose of the robot from a rough initial guess. The right screen shows the ground-truth robot simulation in Gazebo, and the left screen shows our localization alogrithm as visualized in RViz. As seen, the filter quickly and correctly identifies the transformation between the robot's frame and the map, and recovers despite a brief hiccup. This demonstration uses n=1000 particles.


## Our Approach: Particle filter overview

We implemented a particle filter algorithm, a popular and well-studied approach to localization relying on a Monte Carlo paradigm. We provide the algorithm an initial estimate of the robot's pose in the known map. From there, the algorithm scatters a set of "particles" around the initial guess, where each particle is effectively an individual hypothesis for the robot's actual X-Y-theta pose. Every time the algorithm runs, it computes the likelihood that each particle/hypothesis in the map is in the same position as the actual robot. It takes the actual laser scan measurements from the robot and then projects these ranges from each particle into the map, thus identifying the points on the map where the laser would have hit an obstacle if the robot shared that particle's pose; if these projected points tend to fall on obstacles, the particle is likely, and otherwise it is unlikely. Each particle is thus assigned a weight denoting how strong of a candidate it is for the robot's actual position in the map; these weights are normalized so they sum to one and thus form a valid probability distribution.

After every scan comparison and re-weighting, the algorithm "resamples" the particles. This process entails redistributing the particles based on the computed probability distribution, with resampled particles likely to be placed near strong candidates. This has the effect of throwing out poor hypotheses and concentrating computing power on regions of high probability, hopefully without overcommitting to any particular hypotheses in the event that they turn out to be incorrect. After resampling, data from the robot's odometry is used to update each particle's pose, as the algorithm then repeats itself to judge each new particle's likelihood in light of a new laser scan from a new vantage point. Hopefully a group of particles will coalesce around the actual position, always keeping in step with the robot via odometry data and reaffirming their validity at each step via new laser scan data, thus localizing the robot.


## Design Decision: Noisy odometry update

An important element in particle filter design is the source of randomness in one's particle cloud, since the algorithm relies on having a somewhat diverse set of hypotheses for the robot's pose. While converging hypotheses can suggest an accurate pose estimate, too much convergence can make the algorithm fail if it locks on to a single estimate that's not totally correct. Thus, some amount of variability in the updated & resampled poses is key to a successful implementation.

We initially considered introducing this variability purely in the resampling step, where the new particles spawning from high-confidence particles are slightly randomly displaced at creation. However, this approach isn't firmly grounded in any physical or probabilistic justification. Introducing variability in the odometry update step more accurately reflects the nature of the underlying system, as both the robot's odometry measurements and its execution of motor commands are inherently imperfect processes with some degree of uncertainty. We thus chose to use the noisy motion model sampling algorithm presented by Thrun et al, which injects noise into the robot's odometry-based pose update.

![Thrun's noisy motion model](/media/motion_model.png)
*Thrun et al's noisy motion sampling algorithm (left) decomposes each motion segment (right) into two turns with a linear traverse in between. Each of these 3 elements has noise added, with the possible magnitude of noise determined by combinations of the turn & traverse magnitudes and the 4 noise parameters, alpha_1 to 4.* 

This algorithm samples random quantities of noise based on 4 noise parameters, which govern how the magnitudes of the odometry-measured angular turns and linear traverse will impact the magnitude of this noise in the sampled turns and traverse. Thrun offers no guidance on interpreting and setting these parameters, but by paying attention to the units involved we were able to decipher their roles and choose appropriate values. Erring on the side of higher noise parameters seemed like a safe strategy, as in this case the noise mostly exists to prevent overconvergence of the hypothesis set, which we seem to have accomplished. Further, this noisy odometry update let us avoid injecting noise in the resampling step whatsoever, thus giving us a very straightforward and intepretable resampling algorithm.


## Design Decision: Laser scan likelihood

Another key step of our algorithm is estimating the particle likelihoods using laser scan measurements. After projecting the scans onto the map, the `OccupancyField` class reports the distance from each projected location to the nearest obstacle in the map. We thus need a scheme for converting these individual errors to a representation of particle likelihood, where a full set of zero errors corresponds to a maximum-likelihood particle for that laser scan.

Thrun et al. suggest an approach firmly grounded in probability theory, wherein one conditions on the particle being correct (i.e. the laser scan being taken from the particle's pose) and assumes that each laser beam measurement is an independent event. Then the probability of getting the particular array of errors - which can be converted to the probability that the particle is correct - is simply the product of the probabilities of getting each error in isolation.

While this approach is nice in theory, we ran into a few practical issues implementing it. Namely, taking the product across all 360 scans tended to tempt the limits of floating-point precision, especially when dealing with the low probabilities generated by highly-unlikely scans. This tended to make normalization somewhat finicky, and also interacted poorly with our NaN problem described below. We thus decided to pivot towards a sum-of-squared-errors approach, where likely scans accrue little squared error and are thus rewarded when the negative third power of each sum is taken as the probability. While not explicitly probability-theoretical, this scan likelihood approach worked well for us and proved quite robust.


## Design Decision: Particle cloud as Numpy arrays

Initially, we represented each particle as an instance of a `Particle` class. While this did come with a few advantages - such as clean encapsulation of each particle, straightforward access to any given particle's full attribute set, and ease of dynamic resizing of the particle cloud by creating or destroying objects - we realized we weren't making enough use of them to justify the performance overhead. Indeed, this data architecture required us to store the instances in a list and access them via `for` loops, which incur a significant performance penalty in Python. Once we had implemented the algorithm with this initial architecture and wanted to test with increasingly large numbers of particles, we found the performance unsatisfactory, so we refactored to leverage Numpy arrays stored in an instance of a `ParticleCloud` class. Under this architecture, one Numpy array per attribute is used to store the value of that attribute across all particles in the "particle cloud," letting us transform and operate on the particles using Numpy's efficient array operations.

Refactoring our code in this manner was a significant investment, but it certainly paid off. Turning each particle by a distinct angle became as simple as storing these angles as an array and taking an elementwise sum. This architecture proved especially efficient when normalizing the particles' weights and when drawing random samples from a probability distribution. While we do not have quantitative data on the performance improvement, it suddenly let us run upwards of several thousand particles using all 360 laser scans without a noticeable slowdown. This investment tangibly changed the nature of the accuracy/robustness-vs-efficiency tradeoff we'd been dealing with previously, as we were suddenly able to process over an order of magnitude as much data in the same time. We are confident that, with some tuning and testing, this efficiency would let us solve the "robot kidnapping problem": that is, localize on a fairly large map without prior knowledge of the robot's pose.


## Challenge: Out-of-bounds scans

When projecting scans from particles onto the map, one inevitably deals with cases where the projected scan goes off the edge of the map. In such of invalid projections, the literature suggests simply discarding the corresponding scans and not factoring them into the likelihood calculation. However, we did much of our testing in the relaitvely small "gauntlet" world with a map which did not go past the inner surface of its enclosing walls, so we ran into out-of-bounds projected scans very frequently. We knew we were losing out on perfectly valuable information from these out-of-bounds scans. Additionally, it proved cumbersome to filter out the `NaN` measurements resulting from out-of-bounds scans from our Numpy array-based computations. 

We realized we could prevent these out-of-bounds readings by leveraging our knowledge of the "gauntlet" world to pad out the map with a large border of empty space. This was a fairly straightforward operation, but it seemed to break the system in some other way. It turns out we overlooked the need to update the map's origin as determined by its associated metadata file, as the `OccupancyField` class uses this origin to compute distances to projected points. Once we finally fixed this, we achieved sensible distance computations without out-of-bounds scans.


## Challenge: Nasty angle unit conversion bug

For some time, our algorithm simply wasn't behaving sensibly despite so many components working in isolation. We narrowed down the problem to our likelihood estimations from the laser scanner, but still struggled to nail down the problem. It was difficult to use visualization strategies, as we had to spend a good amount of time simply tracking down whether the bug was something geometric (perhaps a transformation?), arithmetic (the sum-of-squares likelihood function?), or algorithmic. This bug stood between us and a working particle filter for far too long.

Ultimately, some very careful print-based debugging on a stripped-down version of the problem (a single particle considering only a handful of scans) revealed that the projected points from each particle were not fanning out around the robot as much as expected, but rather clumped in front of the robot. Closer inspection revealed that an intermediate calculation in the projection code expected scan angles in degrees and thus cast them from degrees to radians. However, these scan angles were already in radians, so the unit conversion squashed all the angles to approximately zero. With this conversion removed, the algorithm worked immediately.

This bug was especially nasty because it involved a ROS message interface defintion and thus fell out of the immediate purview of our Python code. In fact, our function's code adhered to the units listed in the docstring; it was our own documentation that was wrong. Remembering to double-check one's assumptions about inputs & outputs is never a bad habit!


![Particle filter demo in Gauntlet world](/media/gauntlet_demo.gif)


## Potential Improvements

While robust in fairly simple scenarios, our particle filter remains largely unproven in more complex tasks and real-world scenarios. With additional time to test and experiment, we would be curious to push the boundaries of our code to see whether it could solve the robot kidnapping problem in large environments. 

Revisiting the particle likelihood estimation function and resampling step may also be worthwhile. Currently our likelihood estimation is totally ungrounded in probability theory while our resampling is very grounded - perhaps to a fault, in that it does little to inject new particles into the possibility space. We suspect this may be more important when localizing in a larger and perhaps more symmetric space. It would also be interesting to probe some of the assumptions that go into the literature likelihood estimation function - such as the independence of different beams from a laser scanner.

Another useful improvement we could make would be actively intervening when a particle's position is impossible. For example, in our current implementation, a particle can be off the edge of the map and driving happily. Obviously, this is not a possible robot location, and a better algorithm would immediately resample that particle somehow. The easiest approach would be to just randomly distribute the particle anywhere in the map, but we could also randomly assign it around the current pose estimate, or assign it somewhere near where it ran off the edge of the map.

One other interesting route to explore would be experimenting with Python libraries that allow one to attach units to all values, since a simple unit conversion error halted our progress on this project for so long. It would be interesting to see this technology built into more ecosystems to allow units to be robustly handled by the code itself, rather than by the (error-prone) person writing the code.


## Lessons for Future Project

The main procedural lesson learned from this project would be to start earlier, and get the foundations working as soon as possible. We got a bit of a late start, and it took us a while to get the visualization of anything working, which left us with relatively little time to get the particle filter working and tuned.

From a more robotics-specific perspective, visualizations were and continue to be key. Representing the weight of particles in RViz somehow could have made debugging the weighting algorithm much easier, and likewise visualizing a particles' projected scan points could have saved us much headache on the unit conversion bug. In cases where getting things into RViz is daunting, even simpler visualization strategies like using MatPlotLib can prove effective, as sometimes the bug is quite obvious if you can visualize it even just a little.

On a related note, testing our subcomponents in as simple a setting as possible (for example, having one particle) would have made it massively easier to debug them. Additionally writing unit tests alongside the code (or even beforehand as a way to pin down interfaces) for extremely simple cases of our procedures would go a long way to giving us peace of mind that certain components are working as intended, letting us focus debugging time on the unproven portions.
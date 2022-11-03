### Computational Robotics Fall 2022

# Robot Localization Project

### Han Vakil and Luke Raus

### Project Goal

The goal of this project was to implement a particle filter algorithm for robot localization. In practice, this means finding the true position of a robot navigating a space using the onboard odometry, laser scans from the robot, a known map of the space, and an initial estimate of the position of the robot. 

This allows for a robot operating in a known environment to accurately locate itself, a task vital to many robotic operations. We ran our filter on the gauntlet world from QEA2, and a provided map of the first floor of the MAC.

### Solving the Problem

To solve the problem, we implemented a particle filter algorithm. (explain)

### Design Decision

One major design decision in the project was our scan resolution, or how many data points from the laser scanner we wanted to use to judge the "correctness" of a particle. Lower resolution (meaning fewer skipped scan angles) meant we'd get more data to work with, but would significantly decrease the speed of our algorithm running. On the other hand, limiting our algorithm to too few points would limit how much data we'd have, decreasing our ability to effectively judge a particle's correctness. We settled on a resolution of 20 after some rough testing, meaning we used one of every twenty scans as data to process.

### Challenges

Our biggest challenge we faced was our particle filter converging too quickly. It's important in particle filter algorithms to find the right balance of converging on promising points, so you get a more accurate estimate of your robot's position, with exploring semi-random points and points *around* your best guess, so you don't lock in on one potentially inaccurate point just because it had a slightly higher initial confidence than the other points.

We didn't really find a good balance for this. Our particle weighting algorithm seems to assign strongly contrasting weights to particles, meaning particles with slightly more promise end up with very high weights compared to their slightly-less promising compatriots. In practice, this meant the algorithm rapidly converged on these points, ignoring other almost-as-promising candidates. With these points so rapidly converged on and so little variance in the points we considered as possible robot locations, the algorithm would very quickly zero in on a projected location and basically just apply the incoming odometry data to that position without much input from the laser scan. This meant (as you can see in the example) that the algorithm predicts the location drastically wrong, even guessing that the robot is inside a wall which is obviously impossible.

### Improvements

The main improvement we'd need to make to improve this algorithm would be redoing our weighting and adding stronger resampling to prevent the rapid convergence we're seeing now. This would mean we'd converge slower, consider more possible promising points (narrowing down over time of course, but doing that narrowing down more slowly and with more data so we don't potentially disregard good candidates early).

Another useful improvement we could make would be actively intervening when a particle's position is impossible. For example, in our current implementation, a particle can be off the edge of the map and driving happily. Obviously, this is not a possible robot location, and a better algorithm would immediately resample that particle somehow. The easiest approach would be to just randomly distribute the particle anywhere in the map, but we could also randomly assign it around the current pose estimate, or assign it somewhere near where it ran off the edge of the map.

### Lessons for Future Project

The main procedural lesson learned from this project would be to start earlier, and get the foundations working as soon as possible. We got a bit of a late start, and it took us a while to get the visualization of anything working, which left us with little time to get the particle filter working and tuned.

From a more robotics-specific perspective, visualizations were and continue to be key. Representing the weight of particles in rviz somehow could have made debugging the weighting algorithm much easier. On a related note, testing our subcomponents in as simple a setting as possible (for example, having one particle) would have made it massively easier to debug them.

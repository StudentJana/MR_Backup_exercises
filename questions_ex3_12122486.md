# Mobile Robotics
## Questions
* Author: Jana Grabher
* StudentID: 12122486
* Semester: 2026S

### Exercise 3

#### Exercise Questions
##### Coordinate Transformations
Why is it necessary to apply a transformation matrix M to raw laser measurements?
* [TRUE] To translate and rotate the sensors local frame to match the maps global grid.
* [FALSE] To convert raw laser distances from meters directly into integer pixel counts.
* [TRUE] To account for the robots current pose and the sensors physical offset.
* [FALSE] To automatically filter out maximum range measurements.

##### Particle Filter Resampling
What is the primary purpose of the resampling step in a Particle Filter?
* [TRUE] It duplicates particles with high weights and destroys those with low weights.
* [TRUE] Without it, particles will naturally disperse causing all to have near-zero weights.
* [FALSE] It recalculates expected measurements using ray-tracing before the next update.
* [FALSE] It periodically scatters new particles randomly to recover from tracking loss.


#### Lecture Question
##### Beam vs Scan Models
Why is the Scan-based Model generally more computationally efficient than the Beam-based Model?
* [FALSE] The Scan-based Model completely ignores random uniform noise in its calculations.
* [TRUE] The Beam-based Model requires computationally expensive ray-tracing for every beam.
* [TRUE] The Scan-based Model uses a fast lookup in a precomputed distance map.
* [FALSE] The Beam-based uses complex probabilities while the Scan model only uses addition.

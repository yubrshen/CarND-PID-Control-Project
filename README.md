# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Achievement

A PID controller is implemented. A message-driven twiddle procedure for PID parameters is implemented. 
I also did a  [prototype of message-driven twiddle process in Python](./src/twiddler.py) first for proof of concept.
It may serve as a reference for implementation in C++. 

The PID controller with manually selected parameters such as {0.2, 3.0, 0.004} for P, D, I respectively with constant throttle 0.4 can achieve
stable driving the full track with top speed of 45 mph. 

With adaptive throttle rule of $0.7 - \sqrt{fabs(angle)\cdot speed\cdot fabs(cte)}$, through twiddle process a set of paramters were found 
that can enable mostly safe driving with top speed of 70 mph, and most of the time the speed is above 40 mph. There are two incidents where 
that the car drove over to the lanes markings usually forbidden.

Painstaking effort were made to search for parameters for safer driving. But I was only able to minimize the number of incidents of traffic violations. 

In the future, I might try to use another PID to control the throttle. 

I did attempted on the effort, so far with my current 
found formula for ideal speed: $MAX_SPEED \cot (1 - \frac {\sqrt{angle}} {5})$ where $ 5 = \sqrt{25}$ and 25 is considered to be the maximum angle, 
I feel the error signal being the difference between the idea speed given steering angle and the current speed may not capture the 
requirement that the speed at the extreme angle 25 should be very small regardless to the current speed. But I had not found good expression for the ideal speed thus I have not further attempted. 

Here is a capture of the running at high speed after twiddle process:

![](./after-tuning-full-track-20170605.gif)

The following shows the typical twiddle process. 

![](./typical-tuning--session-20170602.gif)

The follow shows only P component in the controller:

![](./p-alonei-20170605.gif)

It will only work when there is not much change in the speed of cte change. 

The follow shows only the D component in the controller:

![](./d-alone-20170605.gif)

It performs even worse, as it does not account for the presence of cte directly, only will account of the change of cte. 

This shows only the I component in the controller:

![](./i-alone-20160605.gif)

It performs the worst, as it only response to the accumulation of cte. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


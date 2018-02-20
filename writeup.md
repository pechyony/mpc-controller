# Implementation of MPC controller

## Motion model and optimization problem
In this section we describe a kinematic model and optimization problem that assume that there is no latency when applying activators. In the next section we describe an extension of the model and optimization problem that takes into account the latency. 

We used a global kinematic model with 6-dimensional state vector

<center>
s<sub>t</sub> = [x<sub>t</sub>, y<sub>t</sub>, &psi;<sub>t</sub>, v<sub>t</sub>, cte<sub>t</sub>, e&psi;<sub>t</sub>]  
</center>
  
where  
  
* x<sub>t</sub> and y<sub>t</sub> are car coordinates at time t 
* &psi;<sub>t</sub> is a direction of velocity at time t  
* v<sub>t</sub> is a velocity at time t  
* cte<sub>t</sub> is a cross-track error at time t, defined as the distance between the desired position (x, f(x)) and the actual position (x<sub>t</sub>, y<sub>t</sub>) of the car. In our code we approximated cte<sub>t</sub> by f(x<sub>t</sub>) - y<sub>t</sub>  
* e&psi;<sub>t</sub> is an error of the velocity direction at time t, defined as &psi;<sub>t</sub> - arctan(f'(x<sub>t</sub>)).
 
The state vector s<sub>t+1</sub> at time t+1 is predicted from the state vector at time t using the following equations:

x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> cos(&psi;<sub>t</sub>) dt  
y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> sin(&psi;<sub>t</sub>) dt  
&psi;<sub>t+1</sub> = &psi;<sub>t</sub> - v<sub>t</sub> &delta;<sub>t</sub> dt / L<sub>f</sub>  
v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> dt  
cte<sub>t+1</sub> = cte<sub>t</sub> + v<sub>t</sub> sin(e&psi;<sub>t</sub>) dt = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> sin(e&psi;<sub>t</sub>) dt  
e&psi;<sub>t+1</sub> = e&psi;<sub>t</sub> - v<sub>t</sub> &delta;<sub>t</sub> dt / L<sub>f</sub> = &psi;<sub>t</sub> - arctan(f'(x<sub>t</sub>)) - v<sub>t</sub> &delta;<sub>t</sub> dt / L<sub>f</sub>

where

* &delta;<sub>t</sub> and a<sub>t</sub> are change of direction and acceleration applied at time t  
* dt is the difference between time t+1 and t  
* L<sub>f</sub> is the distance between the center of mass of the vehicle and it's front axle. We used L<sub>f</sub>=2.67 .

The values of &delta;<sub>t</sub> and a<sub>t</sub> are found by solving an optimization problem. The objective function of this optimization problem has 4 components: 

* **error component**, &sum;<sub>t=0,1,...,N-2</sub> ( C<sub>cte</sub>&bullet;(cte<sub>t</sub>)<sup>2</sup> + C<sub>e&psi;</sub>&bullet;(e&psi;<sub>t</sub>)<sup>2</sup> + C<sub>v</sub>&bullet;(v<sub>t</sub> - v<sub>target</sub>)<sup>2</sup>), that ensures that the car has low cross-track and direction errors and also drives with the velocity that is as close to the desired velocity v<sub>target</sub> as possible. The value of v<sub>target</sub> is a constant positive parameter.

* **actuators component**, &sum;<sub>t=0,1,...,N-2</sub> (C<sub>&delta;</sub>&bullet;(&delta;<sub>t</sub>)<sup>2</sup>+C<sub>a</sub>&bullet;(a<sub>t</sub>)<sup>2</sup>), that ensures that the driving is not wobbly and the velocity is as close to constant as possible.

* **smoothness component**, &sum;<sub>t=0,1,...,N-3</sub>  (C<sub>delta_diff</sub>&bullet;(&delta;<sub>t+1</sub>-&delta;<sub>t</sub>)<sup>2</sup>+C<sub>a_diff</sub>&bullet;(a<sub>t+1</sub>-a<sub>t</sub>)<sup>2</sup>), that ensures that the car drives smoothly and doesn't have abrupt changes of speed and direction.

* **slowdown component**, &sum;<sub>t=0,1,...,N-2</sub> C<sub>slowdown</sub>&bullet;((cte<sub>t</sub>&bullet;a<sub>t</sub>)<sup>2</sup>+(e&psi;<sub>t</sub>&bullet;a<sub>t</sub>)<sup>2</sup>), that reduces acceleration when the car is in dangerous zone and has a large cross-track error and/or large error in the direction of velocity. 

In all these components the parameters C<sub>cte</sub>, C<sub>e&psi;</sub>, C<sub>v</sub>, C<sub>&delta;</sub>, C<sub>a</sub>, C<sub>delta_diff</sub>, C<sub>a_diff</sub>, C<sub>slowdown</sub> have non-negative values. The parameter N is the number of look-ahead steps and is described in detail in the last section.

The final optimization problem is 

min <sub>&delta;<sub>0</sub>,...,&delta;<sub>N-2</sub>,a<sub>0</sub>,...,a<sub>N-2</sub></sub>  &sum;<sub>t=0,1,...,N-2</sub> ( C<sub>cte</sub>&bullet;(cte<sub>t</sub>)<sup>2</sup> + C<sub>e&psi;</sub>&bullet;(e&psi;<sub>t</sub>)<sup>2</sup> + C<sub>v</sub>&bullet;(v<sub>t</sub> - v<sub>target</sub>)<sup>2</sup>) + &sum;<sub>t=0,1,...,N-2</sub> (C<sub>&delta;</sub>&bullet;(&delta;<sub>t</sub>)<sup>2</sup>+C<sub>a</sub>&bullet;(a<sub>t</sub>)<sup>2</sup>) +   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&sum;<sub>t=0,1,...,N-3</sub>  (C<sub>delta_diff</sub>&bullet;(&delta;<sub>t+1</sub>-&delta;<sub>t</sub>)<sup>2</sup>+C<sub>a_diff</sub>&bullet;(a<sub>t+1</sub>-a<sub>t</sub>)<sup>2</sup>) + &sum;<sub>t=0,1,...,N-2</sub> C<sub>slowdown</sub>&bullet;((cte<sub>t</sub>&bullet;a<sub>t</sub>)<sup>2</sup>+(e&psi;<sub>t</sub>&bullet;a<sub>t</sub>)<sup>2</sup>) 

such that

&forall; t=0,...,N-2,  -1 &leq; a<sub>t</sub> &leq; 1  
&forall; t=0,...,N-2,  -0.436332 &leq; &delta;<sub>t</sub> &leq; 0.436332  
&forall; t=0,...,N-2,  x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> cos(&psi;<sub>t</sub>) dt  
&forall; t=0,...,N-2,  y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> sin(&psi;<sub>t</sub>) dt  
&forall; t=0,...,N-2,  &psi;<sub>t+1</sub> = &psi;<sub>t</sub> - v<sub>t</sub> &delta;<sub>t</sub> dt / L<sub>f</sub>  
&forall; t=0,...,N-2,  v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> dt  
&forall; t=0,...,N-2,  cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> sin(e&psi;<sub>t</sub>) dt  
&forall; t=0,...,N-2,  e&psi;<sub>t+1</sub> = &psi;<sub>t</sub> - arctan(f'(x<sub>t</sub>)) - v<sub>t</sub> &delta;<sub>t</sub> dt / L<sub>f</sub>

Notice that the last six constraints are the global kinematic model described above. These constraints describe a trajectory of the car, from the known state s<sub>0</sub> = [x<sub>0</sub>, y<sub>0</sub>, &psi;<sub>0</sub>, v<sub>0</sub>, cte<sub>0</sub>, e&psi;<sub>0</sub>] of the car immediately before solving optimization problem, to the future states s<sub>1</sub>,...,s<sub>N-1</sub>.

We use the values of &delta;<sub>0</sub> and a<sub>0</sub>, found by solving the above optimization problem, to accelerate the car and change its current direction.  

## Model Predictive Control with Latency

In our car there is a latency of 100 milliseconds when we decide to apply the actuators &delta;<sub>0</sub> and a<sub>0</sub> to accelerate the car and change its current direction. In this section we describe an extension of the optimization problem to deal with this latency.

To accomodate the latency, we decompose the time interval [t,t+1] into two parts, [t,t'] and [t',t+1]. The time between t and t' is the latency interval. The time between t' and t+1 is the new value of dt. When applying the new actuators &delta;<sub>0</sub> and a<sub>0</sub> at time t, the car continues to drive at time [t,t'] with the old values of actuators. The new actuators are only applied at time t'. 

Since each time iterval [t,t+1] is split into two parts, we replace N with 2N-1 lookahead steps. The time between each even step and its successive odd step is the latency interval. The new actuators are found at even steps, but applied at the odd steps and last for two time steps. Hence we define dt<sub>i</sub> to be dt when i is odd and latency interval when i is even. The modified optimization problem is   

min <sub>&delta;<sub>0</sub>,...,&delta;<sub>N-2</sub>,a<sub>0</sub>,...,a<sub>N-2</sub></sub>  &sum;<sub>t=0,1,...,2N-1</sub> ( C<sub>cte</sub>&bullet;(cte<sub>t</sub>)<sup>2</sup> + C<sub>e&psi;</sub>&bullet;(e&psi;<sub>t</sub>)<sup>2</sup> + C<sub>v</sub>&bullet;(v<sub>t</sub> - v<sub>target</sub>)<sup>2</sup>) + &sum;<sub>t=0,1,...,N-2</sub> (C<sub>&delta;</sub>&bullet;(&delta;<sub>t</sub>)<sup>2</sup>+C<sub>a</sub>&bullet;(a<sub>t</sub>)<sup>2</sup>) +   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&sum;<sub>t=0,1,...,N-3</sub>  (C<sub>delta_diff</sub>&bullet;(&delta;<sub>t+1</sub>-&delta;<sub>t</sub>)<sup>2</sup>+C<sub>a_diff</sub>&bullet;(a<sub>t+1</sub>-a<sub>t</sub>)<sup>2</sup>) + &sum;<sub>t=0,1,...,N-2</sub> C<sub>slowdown</sub>&bullet;((cte<sub>t</sub>&bullet;a<sub>t</sub>)<sup>2</sup>+(e&psi;<sub>t</sub>&bullet;a<sub>t</sub>)<sup>2</sup>) 

such that

&forall; t=0,...,N-2,  -1 &leq; a<sub>t</sub> &leq; 1  
&forall; t=0,...,N-2,  -0.436332 &leq; &delta;<sub>t</sub> &leq; 0.436332  
&forall; t=0,...,2N-1,  x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> cos(&psi;<sub>t</sub>) dt<sub>i</sub>  
&forall; t=0,...,2N-1,  y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> sin(&psi;<sub>t</sub>) dt<sub>i</sub>   
&forall; t=1,...,2N-1,  &psi;<sub>t+1</sub> = &psi;<sub>t</sub> - v<sub>t</sub> &delta;<sub>&LeftFloor;t/2&RightFloor;-1</sub> dt<sub>i</sub>  / L<sub>f</sub>  
&forall; t=1,...,2N-1,  v<sub>t+1</sub> = v<sub>t</sub> + a<sub>&LeftFloor;t/2&RightFloor;-1</sub> dt<sub>i</sub>   
&forall; t=0,...,2N-1,  cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> sin(e&psi;<sub>t</sub>) dt<sub>i</sub>   
&forall; t=1,...,2N-1,  e&psi;<sub>t+1</sub> = &psi;<sub>t</sub> - arctan(f'(x<sub>t</sub>)) - v<sub>t</sub> &delta;<sub>&LeftFloor;t/2&RightFloor;-1</sub> dt<sub>i</sub>  / L<sub>f</sub>

Notice that the fifth, sixth and eighth constraints now start from t=1 instead of t=0. That's because, due to the latency, at time t=0 these constraints use the current &delta;<sub>curr</sub> and a<sub>curr</sub> direction and acceleration of the car, that are found when solving optimization problem at previous iteration. Hence the optimization problem has three additional constraints:

&psi;<sub>1</sub> = &psi;<sub>0</sub> - v<sub>0</sub> &delta;<sub>curr</sub> dt / L<sub>f</sub>  
v<sub>1</sub> = v<sub>0</sub> + a<sub>curr</sub> dt   
e&psi;<sub>1</sub> = &psi;<sub>0</sub> - arctan(f'(x<sub>0</sub>)) - v<sub>0</sub> &delta;<sub>curr</sub> dt / L<sub>f</sub>

Similarly to the optimization problem in the previous section, the state of the car s<sub>0</sub> = [x<sub>0</sub>, y<sub>0</sub>, &psi;<sub>0</sub>, v<sub>0</sub>, cte<sub>0</sub>, e&psi;<sub>0</sub>] immediately before solving the optimization problem is known.

After tuning parameters, we set N = 5, dt = 0.11, C<sub>cte</sub> = 200, C<sub>e&psi;</sub> = 200, C<sub>v</sub> = 5, C<sub>delta</sub> = 100, C<sub>a</sub> = 100, C<sub>delta_diff</sub> = 200, C<sub>a_diff</sub> = 10, C<sub>slowdown</sub> = 500 and v<sub>target</sub>=100. In the last section we discuss further the influence of N and dt on car's driving.

## Polynomial Fitting and MPC preprocessing
At each iteration we receive from the simulator the car's current state [x<sub>car</sub>, y<sub>car</sub>, &psi;<sub>car</sub>, v<sub>car</sub>] in a global coordinate system. We performed a number of preprocessing steps before solving optimization problem: 

* **Coordinate transformation.** Since the visualization of waypoints and predicted trajectory is done in car coordinate system, we decided to use this coordinate system in our optimization process. The origin of car coordinate system is at car location, x axis is aligned with the direction of velocity and y axis points to the left. Since simulator sends us waypoints in a global coordinate system, we transformed waypoints to car coordinate system. Let (x,y) be  waypoint global coordinates.  We denote by (x',y') the same waypoint with coordinates in car's coordinate system. We used the following equations to transform waypoints to car coordinate system:

    x' = (x-x<sub>car</sub>)&bullet;cos(-&psi;) - (y-y<sub>car</sub>)&bullet;sin(-&psi;)  
    y' = (x-x<sub>car</sub>)&bullet;sin(-&psi;) + (y-y<sub>car</sub>)&bullet;cos(-&psi;)

* **Polynomial fitting**. We fitted a second degree polynomial to waypoints in car's coordinate system. By decreasing polynomial degree from 3 to 2 we reduced processing time at each iteration, while still obtaining a smooth curve that goes through waypoints. We discuss a connection between processing time and the accuracy of car driving in the next section.

* **Initialization of state vector**. After fitting polynomial, we initialize car's state vector [x<sub>0</sub>, y<sub>0</sub>, &psi;<sub>0</sub>, v<sub>0</sub>, cte<sub>0</sub>, e&psi;<sub>0</sub>] in its own coordinate system. By the definition of car coordinate system, x<sub>0</sub>=y<sub>0</sub>=&psi;<sub>0</sub>=0. The value of v<sub>0</sub>=v<sub>car</sub> remains the same in global and car coordinate systems. 

  Let f(x) be a polynomial fitted at the previous step. The value of cte<sub>0</sub> should be the distance from (0,0) to the closest point of the polynomial. The computation of this distance is not straightforward, will increase the processing time, which in turn will affect car's driving. Instead of computing cross-track error exactly, we approximated cte<sub>0</sub> by f(0). 

  The desired direction at point x=0 is arctan(f'(0)). Since the direction of the car in car coordinate system is 0, e&psi;<sub>0</sub> = 0 - arctan(f'(0)) = -arctan(f'(0)). 

## Timestamp length and elapsed duration (N & dt)

In this section we describe our reasoning for choosing the values of N and dt. 

The parameter N defines the number of predictions when building car's trajectory and solving optimization problem. The optimization time is proportional to N. A small value of N (e.g. 3,4) would reduce processing time, but will make projected trajectory noisy and less reliable. A large value of N (e.g. 10) will result in an accurate prediction of car trajectory, but will also increase the processing time. We observed emprically that the large processing time causes car to react slowly to sharp turns, which in turn might put the car on ledges or off-track.

The parameter dt defines the length of prediction interval. When dt is small (e.g. less than 0.1) the car drives wobbly. Also when dt is large (e.g. 0.2) the car reacts slowly to sharp turns, which in turn might put the car on ledges or off-track.

Based on these considerations, after several trials we choose N=5 and dt=0.11.
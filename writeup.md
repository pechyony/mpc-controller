# Implementation of MPC controller

## The model
We used a global kinematic model with 6-dimensional state vector

<center>
<i>s<sub>t</sub></i> = [<i>x<sub>t</sub></i>, <i>y<sub>t</sub></i>, <i>&psi;<sub>t</sub></i>, <i>v<sub>t</sub></i>, <i>cte<sub>t</sub></i>, <i>e&psi;<sub>t</sub></i>]
</center>

where  
<i>x<sub>t</sub></i> and <i>y<sub>t</sub></i> are car coordinates at time <i>t</i> 
<i>&psi;<sub>t</sub></i> is a direction of velocity at time <i>t</i>  
<i>v<sub>t</sub></i> is a velocity at time <i>t</i>  
<i>cte<sub>t</sub></i> is a cross-track error at time <i>t</i>, defined as the distance between the desired position (<i>x</i>, <i>f(x)</i>) and the actual position (<i>x<sub>t</sub></i>, <i>y<sub>t</sub></i>) of the car. In our code we approximated <i>cte<sub>t</sub></i> by <i>f(x<sub>t</sub>) - y<sub>t</sub></i>.   
<i>e&psi;<sub>t</sub></i> is an error of the velocity direction at time <i>t</i>, defined as <i>&psi;<sub>t</sub> - </i>arctan(<i>f'(x<sub>t</sub>)</i>)
 
The state vector <i>s<sub>t+1</sub></i> at time <i>t+1</i> is predicted from the state vector at time <i>t</i> using the following equations:

<i>x<sub>t+1</sub></i> = <i>x<sub>t</sub></i> + <i>v<sub>t</sub></i> cos(<i>&psi;<sub>t</sub></i>) d<i>t</i>  
<i>y<sub>t+1</sub></i> = <i>y<sub>t</sub></i> + <i>v<sub>t</sub></i> sin(<i>&psi;<sub>t</sub></i>) d<i>t</i>  
<i>&psi;<sub>t+1</sub></i> = <i>&psi;<sub>t</sub></i> - <i>v<sub>t</sub> &delta;<sub>t</delta></i> d<i>t</i> / <i>L<sub>f</sub></i>  
<i>v<sub>t+1</sub></i> = <i>v<sub>t</sub></i> + <i>a<sub>t</sub></i> d<i>t</i>  
<i>cte<sub>t+1</sub></i> = <i>cte<sub>t</sub></i> + <i>v<sub>t</sub></i> sin(<i>e&psi;<sub>t</sub></i>) d<i>t</i> = <i>f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub></i> sin(<i>e&psi;<sub>t</sub></i>) d<i>t</i>  
<i>e&psi;<sub>t+1</sub></i> = <i>e&psi;<sub>t</sub></i> - <i>v<sub>t</sub> &delta;<sub>t</delta></i> d<i>t</i> / <i>L<sub>f</sub> = <i>&psi;<sub>t</sub> - </i>arctan(<i>f'(x<sub>t</sub>)</i>) - v<sub>t</sub> &delta;<sub>t</delta></i> d<i>t</i> / <i>L<sub>f</sub></i>

where

<i>&delta;<sub>t</sub></i> and <i>a<sub>t</sub></i> are change of direction and acceleration applied at time <i>t</i>  
d<i>t</i> is the difference between time <i>t+1</i> and <i>t</i>  
<i>Lf</i> is the distance between the center of mass of the vehicle and it's front axle. 

The values of <i>&delta;<sub>t</sub></i> and <i>a<sub>t</sub></i> are found by solving the following optimization problem:


## Model Predictive Control with Latency

## Polynomial Fitting and MPC preprocessing

## Timestamp length and elapsed duration (N & dt)


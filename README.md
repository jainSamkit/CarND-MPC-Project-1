# Model Predictive Control for the Udacity Simulator

The project consist of the c++ implementation of the car to predict the steering angle and the throttle values for the Udacity simulator.The project uses MPC controller to optimise the actutors to be used in the car for particular time period.The actuators are steering angle and the throttle.


## 1.Files

Thr following files depict the work to be done:
* MPC.cpp:   MPC controller class definition;
* main.cpp:  main file that integrates the MPC controller with the simulator.

## 2 Dependency

This project requires the following packages to work:
* Udacity Simulator [https://github.com/udacity/self-driving-car-sim/releases/](https://github.com/udacity/self-driving-car-sim/releases/);
* cmake 3.5 or above;
* make 4.1 or above;
* gcc/g++: 5.4 or above;
* uWebSocketIO;
* ipopt;
* CppAd.

## 3.1 Compiling and Running

The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid
6. Run the Udacity Simulator (./term2_simulator)

## 4.Model Predictive Control

The MPC is an advanced control algorithm to solve the control problems and get the optimsed control outputs.The MPC solves an optimisation problem by minimizing the cost function based on the prediction algorithm or the control model.The cost function in our case is the total sum of cross-track error and the heading error over a particular set of horizons.The predcition algorithm is the bicycle model in that case.

### 4.1 Prediction Horizon

The prediction horizon is the timesteps over which the predictions are made.The timstep in our case is depicted by the the variable "N" as the number of timesteps which also depict the number of timesteps to be used.The variable dt is the time to be taken between each timestep.

Higher N will also result in more complexity and higher dt will lead to imperfect approximation. In case ,when vehicle has higher speeds ,the product of N and dt should be low as the car has less time to fix up the errors .Therefore the predictions should be made on less durations. 

To tweak N and dt ,we have used trial and error by driving the car on the simulator for particular values of N and dt.
The N is 12 and the dt is fixed to 0.03 second.

### 4.2 State

The state consist of the vector ``['x','y','psi','c','cte','epsi']``.The x ad y here are the coordinates of the car ,psi is the heading and v the velocity.The cte is the cross track error and the epsi is the orientation error. There are also the actuator inputs which consist of two elements that is throttle and steering abgle.

### 4.3 Model (Update equations)


![equation](http://latex.codecogs.com/gif.latex?x_%28t&plus;1%29%20%3D%20x_t%20&plus;%20v_t%20*%20cos%28%5Cpsi_t%29*dt)

![equation](http://latex.codecogs.com/gif.latex?y_%28t&plus;1%29%20%3D%20y_t%20&plus;%20v_t%20*%20sin%28%5Cpsi_t%29*dt)

![equation](http://latex.codecogs.com/gif.latex?%5Cpsi%20_%28t&plus;1%29%20%3D%20%5Cpsi%20_t%20&plus;%20%5Cfrac%7Bv_t%7D%7BL_f%7D*%20%5Cdelta_t%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?v_%28t&plus;1%29%20%3D%20v%20_t%20&plus;%20a_t%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?cte_%28t&plus;1%29%20%3D%20f%28x_t%29%20-%20y_t%20&plus;%20v%20_t%20*%20sin%28e%5Cpsi%20_t%29%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi%20_%28t&plus;1%29%20%3D%20%5Cpsi%20_t%20-%20%5Cpsi%20dest%20&plus;%20%5Cfrac%7Bv_f%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)



### 4.4 Coordinate Transform

The whole coordinate system is rotated by the angle psi to make the orientation and the position of the car the origin so that further work could be transformed to the car coordinates to simplify the further work.I have provided the code below: 

```c
	for(unsigned int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = shift_x * cos(0 - psi) - shift_y * sin(0 - psi);
            ptsy[i] = shift_x * sin(0 - psi) + shift_y * cos(0 - psi);
          }
          ```

``Lf`` is the distance between the cars front and the COM of the car.
### 4.5 Latency

We take into account the latency i.e. the time taken by actuators to reflect the real throttle and the steering angle.
```c
dt = 0.1;
x1    = v * cos(0) * dt;
y1    = v * sin(0) * dt;
psi1  = - v/Lf * steer_value * dt;
v1    = throttle_value * dt;
cte1  =   v * sin(epsi1) * dt;
epsi1 = - v * steer_value / Lf * dt;
```

The project successfully implements the MPC control algorithm to precict the actuator inputs such as steering and the throttle to drive the car in the simulator.The car drives at a moderate speed close to 40 kmph. Improvements can be made in the whole system by carefully tuning the horizon and the timestep .However the latency should be the first thing to be taken care of as it leads to massive cross track error and orientation error.


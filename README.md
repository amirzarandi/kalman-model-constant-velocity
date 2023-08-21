# Kalman Filtering: <br> Tracking Object Moving with Constant Velocity The Model 

## 1. Problem Statement

Our goal is to track the location (and velocity) of a moving object, e.g. a car, in a 2-dimensional space. The only information available to us is the initial location (and velocity) and a series of noisy measurements of the velocity as the object moves in space. The key assumption of this problem is that the true velocity of the object is known to be a constant. However, the constant velocity is not known to us.

## 2. State Space

We keep track of the following:

1. $x_{1}$ : location in the $x_{1}$ direction.
2. $x_{2}$ : location in the $x_{2}$ direction.
3. $v_{1}$ : velocity in the $x_{1}$ direction.
4. $v_{2}$ : velocity in the $x_{2}$ direction.

Therefore, let the state vector be represented as:

$$
x=\left(\begin{array}{l}
x_{1} \\
x_{2} \\
v_{1} \\
v_{2}
\end{array}\right)
$$

## 3. Kalman Equations and Dynamic Model

### 3.1. Kalman Equations

Reproduced from Wikipedia (below) we have the systems of equations (in matrix format) that need to be solved as part of the Kalman Filtering algorithm. Our challenge is to adapt the problem setting to result in the model that satisfies the form of these equations which will allow us to use Kalman Filtering to track the object's position and velocity.

The Kalman filter model assumes the true state at time $\mathrm{k}$ is evolved from the state at $(\mathrm{k}-1)$ according to:

$$
\mathbf{x}_{k}=\mathbf{A}_{k} \mathbf{x}_{k-1}+\mathbf{B}_{k} \mathbf{u}_{k}+\mathbf{w}_{k}
$$

where,

- $A_{k}$ is the state transition model which is applied to the previous state $X_{k-1}$
- $B_{k}$ is the control-input model which is applied to the control vector $u_{k}$ (not relevant in this example);
- $w_{k}$ is the process noise which is assumed to be drawn from a zero mean multivariate normal distribution with covariance $Q_{k}$ :

$$
\mathbf{w}_{k} \sim \mathcal{N}\left(0, \mathbf{Q}_{k}\right)
$$

At time $k$ an observation (or measurement) $z_{k}$ of the true state $x_{k}$ is made according to:

$$
\mathbf{z}_{k}=\mathbf{H}_{k} \mathbf{x}_{k}+\mathbf{v}_{k} \mathbf{z}_{k}
$$

where $H_{k}$ is the observation model which maps the true state space into the observed space and $v_{k}$ is the observation noise which is assumed to be zero mean Gaussian white noise with covariance $R_{k}$.

$$
\mathbf{v}_{k} \sim \mathcal{N}\left(0, \mathbf{R}_{k}\right)
$$

The initial state, and the noise vectors at each step $\left\{x_{0}, w_{1}, \ldots, w_{k}, v_{1} \ldots v_{k}\right\}$ are all assumed to be mutually independent.

### 3.2. Filtering

The state of the Kalman filter is represented by two variables:

$\hat{\mathbf{x}}_{k \mid k}$, the a posteriori state estimate at time $\mathrm{k}$ given observations up to and including at time $\mathrm{k}$;

$\mathbf{P}_{k \mid k}$, the a posteriori error covariance matrix (a measure of the estimated accuracy of the state estimate).

The Kalman filter can be written as a single equation, however it is most often conceptualized as two distinct phases: "Predict" and "Update". The predict phase uses the state estimate from the previous timestep to produce an estimate of the state at the current timestep. This predicted state estimate is also known as the a priori state estimate because, although it is an estimate of the state at the current timestep, it does not include observation information from the current timestep. In the update phase, the current a priori prediction is combined with current observation information to refine the state estimate. This improved estimate is termed the a posteriori state estimate.

Typically, the two phases alternate, with the prediction advancing the state until the next scheduled observation, and the update incorporating the observation.

#### 3.2.1. Predict

1. Predicted (a priori) state estimate: $\hat{\mathbf{x}}_{k \mid k-1}=\mathbf{A}_{k} \hat{\mathbf{x}}_{k-1 \mid k-1}+\mathbf{B}_{k} \mathbf{u}_{k}$
2. Predicted (a priori) estimate covariance: $\mathbf{P}_{k \mid k-1}=\mathbf{A}_{k} \mathbf{P}_{k-1 \mid k-1} \mathbf{A}_{k}^{T}+\mathbf{Q}_{k}$

#### 3.2.2. Update

1. Innovation or measurement residual: $\tilde{\mathbf{y}}_{k}=\mathbf{z}_{k}-\mathbf{H}_{k} \hat{\mathbf{x}}_{k \mid k-1}$
2. Innovation (or residual) covariance: $\mathbf{S}_{k}=\mathbf{H}_{k} \mathbf{P}_{k \mid k-1} \mathbf{H}_{k}^{T}+\mathbf{R}_{k}$
3. Optimal Kalman gain: $\mathbf{K}_{k}=\mathbf{P}_{k \mid k-1} \mathbf{H}_{k}^{T} \mathbf{S}_{k}^{-1}$
4. Updated (a posteriori) state estimate: $\hat{\mathbf{x}}_{k \mid k}=\hat{\mathbf{x}}_{k \mid k-1}+\mathbf{K}_{k} \tilde{\mathbf{y}}_{k}$
5. Updated (a posteriori) estimate covariance: $\mathbf{P}_{k \mid k}=\left(I-\mathbf{K}_{k} \mathbf{H}_{k}\right) \mathbf{P}_{k \mid k-1}$

### 3.3. Constant Velocity Model

In our constant velocity model, we have the following:

#### 3.3.1. Dynamic Matrix, $A_{k}$

The Dynamic Matrix, $A_{k}$, is calculated from the dynamics of the motion:

$$
\begin{gathered}
x_{1, k+1}=x_{1, k}+v_{1, k} \cdot \Delta t \\
x_{2, k+1}=x_{2, k}+v_{2, k} \cdot \Delta t \\
v_{1, k+1}=v_{1, k} \\
v_{2, k+1}=v_{2, k}
\end{gathered}
$$

Hence,

$$
A_{k}=\left(\begin{array}{cccc}
1.0 & 0.0 & \Delta t & 0.0 \\
0.0 & 1.0 & 0.0 & \Delta t \\
0.0 & 0.0 & 1.0 & 0.0 \\
0.0 & 0.0 & 0.0 & 1.0
\end{array}\right)
$$

$\Delta t$ specifies the time between consecutive steps.

#### 3.3.2. Matrix $B_{k}$ and vector $u_{k}$

In this constant veocity model we have $B_{k}=0$ and $u_{k}=0$.

#### 3.3.3. Observation Matrix, $H_{k}$

We always observe the velocity, $v_{1}$ and $v_{2}$, which results in the following simple model for $H_{k}$ :

$$
H_{k}=\left(\begin{array}{cccc}
0.0 & 0.0 & 1.0 & 0.0 \\
0.0 & 0.0 & 0.0 & 1.0
\end{array}\right)
$$

#### 3.3.4. Measurement Noise Covariance, $R_{k}$

We have the following model for measurement noise covariance matrix:

$$
R_{k}=\left(\begin{array}{cc}
r & 0.0 \\
0.0 & r
\end{array}\right)
$$

The constant $r$ denotes the belief about the variance of the measurement noise.

#### 3.3.5. Process Noise Covariance, $Q_{k}$

Process noise refers to the modeling of forces that could influence our constant velocity model by creating noise in the form of acceleration disturbance. According to Wikipedia, for this constant velocity model we have:

$$
Q_{k}=G \cdot G^{T} \cdot \sigma_{v}^{2}
$$

where,

$$
G=\left[\begin{array}{llll}
0.5 \Delta t^{2} & 0.5 \Delta t^{2} & \Delta t & \Delta t
\end{array}\right]
$$

and we can assume the acceleration process noise $\sigma_{v}^{2}=8.8 \mathrm{~m} / \mathrm{s}^{2}$, according to Schubert, R., Adam, C., Obst, M., Mattern, N., Leonhardt, V., Wanielik, G. (2011). Empirical evaluation of vehicular models for ego motion estimation. 2011 IEEE Intelligent Vehicles Symposium (IV), 534-539.

## 4. Source

Material for this case study was inspired by the following: https://balzer82.github.io/Kalman/

For more details on Kalman Filtering equations used, please check: https://en.wikipedia.org/wiki/Kalman_filter

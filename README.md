# Learning Model Predictive Control with Error Dynamics Regression for Autonomous Racing
Date: 1/18/2025  
By: Sialoi Taa

> [!NOTE]
> Markdown file might look different on GitHub as the environment used in writing the latex equations are in a different environment as GitHub's rendering method. It is highly encouraged to clone the repo or relevant files to your local machine and look at the markdown files through a text editor such as VS Code.


## Table of Contents
- [Problem](#problem)
- [Solution](#solution)
- [Break Down](#break-down)
- [Final Thoughts](#final-thoughts)
- [Notes](#notes)

## Problem


## Solution


## Break Down


## Final Thoughts


## Notes
## I. INTRODUCTION
- **What are Model Predictive Control (MPC) techniques?**   
    *Model Predictive Control (MPC) is a control strategy that uses a mathematical model of a system to predict its future behavior and optimize its performance over a defined time horizon.MPC relies on a model of the system (e.g., physics-based or data-driven models) to predict the system's future state over a finite prediction horizon. The model describes how inputs affect the system's state and outputs.*  

- **What does this work hope to solve?**  
    "*There's a chicken-and-egg problem for autonomous racing research at the handling limit: to safely achieve higher speed levels requires more vehicle dynamics data; but to acquire highspeed data requires the controller to achieve higher speed levels in the first place.*"  
    This means that the problem at hand was to achieve a way to create a controller that can safely handle highspeeds, however to create that controller, higher speeds had to have been achieved already and vehicle dynamic data had to have already of been recorded during times of highspeed. This creates a problem of what comes first the scarcity of data at highspeeds or the absence of a model that can obtain data safely at highspeeds.

- **What was the solution proposed?**  
    "*Therefore, the motivation of this work is to break the circular problem mentioned earlier by performing incremental and safety-aware exploration toward the limit of handling and iterative learning of the vehicle dynamics in high-speed domains.*"  
    The solution proposed was to incrementally record data, update the model and repeat until the limit of the model's system is reached and cannot keep up with the high velocity of the vehicle anymore. The reason why this is great is because this solution provides a way of tuning parameters that compensates the context of the scarcity of high speed data and corrects the problem of having a model that can take the vehicle to higher speeds to obtain that information to update the model. The iterative process of learning and testing the limit of handling as the speed is increased is how the solution fixes the circular problem.  

## Section II:  PROBLEM FORMULATION
- Using a state vector $\mathbf{x} = [v_x, v_y, \omega_z, e_\psi, s, e_y]^\top$ and input vector $\mathbf{u} = [a, \delta]^\top$, they start to build their model:
    - $v_x$: Longitudinal velocity of the vehicle at the center of gravity.
    - $v_y$: Lateral velocity of the vehicle at the center of gravity.
    - $\omega_z$: Yaw rate (angular velocity about the vertical axis) at the center of gravity.
    - $e_\psi$: Heading error, the difference between the vehicle's orientation and the orientation of the path tangent at the closest point.
    - $s$: Path progress, the arc-length of the closest point on the reference path.
    - $e_y$: Lateral deviation from the path, the shortest distance between the vehicle and the reference path.

- Input vector:
    - $a$: Acceleration input (longitudinal control).
    - $\delta$: Steering angle input (lateral control).

- $\tau$ describes the circuit path and maps a range $[0, L]$ onto the $R^2$ space. With $s ∈ [0, L]$, we can use $s$ as input parameters into $\tau$ to map arc length progression into global position such as $(x, y)$.
    - $s ∈ [0, L]$
    - $\tau : [0, L] \rarr R^2$

- $\overline{\tau} = \tau(mod(s, L))$ for $s ≥ 0 $ is the definition for the closed circuit that the kart will be driving on. The reason why this is necessary is because after the first lap, $s$ will continue to grow. Since the arc is a closed circuit of length $L$, we can reset the progression of growth after $s=L$. This will simplify the calculations as we can provide a definite amount of space that we know will be traveresed.

- If the global position of the vehicle is $p = (x, y)$, then the we can model the progression of the vehicle through $s(p) = arg min_s\space||\tau(s)-p||_2$. The point of this equation is to take every value of $s$ inside the range $[0, L]$ and return the point $s$ that is closest to $p$.

- Next to calculate the lateral deviation from the path we use $e_y(p) = min_s\space||\tau(s)-p||_2$. What this equation is going to do is look for the point $s$ that is closest to the point $p$ and return the distance between global position of point $s$, ($\tau(s)$), and point $p$. This is important because we need to know how to calculate the current state of the vehicle from the nominal path. Nominal path is the reference path or *ideal path* that will be the base for the cost function and the kart will try to mimic the most.

- The last variable to calculate for the state vector $x$ is $e_\psi$. To calculate this we use this formula: $e_\psi(p, \psi) = \psi - arctan(\tau'(s(p)))$. $s(p)$ will calculate the point $s$ on the arc that is closest to the vehicle position $p$. $\tau'$ is the derivative of $\tau$, and since $\tau$ is the formula that will turn the arc length position $s$ into a global position, $\tau'$ will calculate the tangent line at arc length $s$. Now finding the $arctan(\tau')$ will give the orientation of the tangent line. If $\psi$ represents the orientation of the vehicle, finding the difference between the orientation of the vehicle and the tangent line will be the error in orientation ($e_\psi$).

- We write the discretize nonlinear dynamics as:  

$$
\begin{align}
    x_{k+1} = f(x_k, u_k)
\end{align}
$$

- This will model the prediction of a future state as a function of a present state and input vector. The following are the constraints placed on the system's state vectors and the input vectors.  
$\mathcal{X} = {x\space\space|\space −W/2 ≤ e_y ≤ W/2}$  
$\mathcal{U} = {u\space\space|\space u_l ≤ u ≤ u_u}$

- The paper then moves onto the cost function:

$$
\begin{align}
    T &= \min_{u_0, u_1, ...}& &\sum^{∞}_{k=0} \mathcal{1}_{\mathcal{F}}(x_k) \tag{2a} \\
    &\text{subject to }& &x_0=\overline{x},  \tag{2b} \\
        &&&x_{k+1} = f(x_k, u_k), \tag{2c} \\
        &&&x_k \in \mathcal{X}, u_k \in \mathcal{U}, \tag{2d} \\
\end{align}
$$


where $\overline{x} \in \mathcal{S} = \{ x\space|\space s=0 \}$ is the state at the start of the track and $\mathcal{F} = \{x\space|\space s \ge L\}$ is the set of states beyond the finish line of the circuit. If for some time step $k$, $x_k \in \mathcal{F}$ for the first time, then the vehicle has finished the lap and has transitioned onto the next lap. $\mathcal{1_F}$ is the indicator function
for the set F, which is defined as:  

$$\begin{align*}
    \mathcal{1_F}(x) = 
    \begin{cases}
        0 &\text{if } x \in \mathcal{F} \\
        1 &otherwise
    \end{cases}
\end{align*}$$

- This indicator function will add 1 to the lap time T that we will focus on minimizing. However, this still doesn't solve the infinite horizon problem which is that the time for a lap to complete could be infinite and lead to optimization becoming impossible. There's also a possibility that the vehicle model function in (2c) could be vastly different from the true vehicle dynamics and lead to disastrous effects, especially at high speeds where the vehicle is at the limits of operation.
- The solution proposed was to create a finite-horizon convex approximation of (2), which uses state and input trajectories from prior laps to synthesize a convex terminal cost-to-go function and target set.
- **Convex terminal cost-to-go function** is a function that penalizes the difference of the end state of the system to the goal state at the end of the lap for the defined finite horizon.
- **Convex target set** means that the set is convex, i.e., for any two points in the set, the line segment joining them is also within the set. 

## Section III:  LOCAL LEARNING MODEL PREDICTIVE CONTROL

- The main idea of LMPC is to use the data from iterations 1 to j − 1 to synthesize a finite-horizon local convex approximation of (2) for iteration j, which is then solved in a receding horizon manner.
- The LMPC creates a control policy that will look at a desired state trajectory and create a list of control inputs $u$ that will get to that desired state.
- Let's talk about how we could create that local convex approximation. Let's say that for iteration $j$ we have a dataset $\mathcal{D}^j\space =\space (\text{x}^j, \text{u}^j)\space \cup \space\mathcal{D}^{j-1} $, where $\text{x}^j = \{x^j_0, x^j_1, ..., x^j_{T^j}\}$ and $\text{u}^j = \{u^j_0, u^j_1, ..., u^j_{T^j}\}$ are the closed-loop state and input sequence for iteration $j$ and $T^j$ denotes the time step at which the lap was completed for iteration $j$, i.e., the first time step where $\mathcal{1_F}(x_k) = 0$. This means that for the current iteration or lap $j$, the dataset of input controls $\text{u}^j$ and state vectors $\text{x}^j$ are combined with the datasets of the previous iterations or laps. 
- Dataset $\mathcal{D}^0$ can be intialized with human driven laps.

### A. Local Convex Target Set
- As the closed-loop trajectories stored in $D^j$ are from the actual vehicle, it is straightforward to see that if we can control the vehicle to any state in $D^j$, then there exists a known feasible control sequence which can be used to complete the lap from that state. 
- Now we talk about how to construct convex terminal sets that are local about a given state $x$. Using $K$-Nearest Neighbors, we look at the past $P$ laps and choose $K$ number of states that are closest to the current state. This is done because it reduces the computational complexity and responds more favorably for fast online control. We choose the target state through evaluating the weighted euclidean distance over the relevant states:  

$$\begin{align*}
    (x^i_k-x)^\top D(x^i_k-x),\text{ } &\forall i \in \{j, j-1, ..., j-(P-1)\}, \\
    &k \in \{0, 1, ..., T^i\} \tag{3}
\end{align*}$$

where P and D ⪰ 0 which means that matrix P and D are positive semidefinite.  
- Then we let $\mathbf{X}^j(x;\mathcal{D}^j) \in \mathbb{R}^{n\times KP}$ denote the matrix formed by the $KP$ states closest to $x$. We then define the target set as the convex hull of these states:  

$$\mathcal{X}^j_N(x;\mathcal{D}^j) = \{ \overline{x}\in\mathbb{R}^{n}\space | \space \exists\lambda\in\mathbb{R}^{KP}, \space 0\le\lambda\le 1,\space \mathbf{1}^\top\lambda=1,\space \mathbf{X}^j(x;\mathcal{D}^j)\lambda = \overline{x} \}$$

- The equation is saying the matrix $\mathcal{X}$ for iteration $j$ and for the finite horizon with $N$ steps uses state $x$ and dataset $\mathcal{D}^j$ for lap $j$ to create state vectors $\overline{x}$ that live inside the space $\mathbb{R}^n$. To find an $\overline{x}$ state vector, we need to find a $\lambda$ that exists in $\mathbb{R}^{KP}$. This lambda vector $\lambda$ has to have elements between 0 and 1, including 0 and 1 too, and no matter what each element is between 0 and 1, all elements must sum up to 1. Once we find such a $\lambda$, we will multiply it with the matrix $\mathbf{X}^j(x;\mathcal{D}^j)$ which will look like $\mathbf{X}^j(x;\mathcal{D}^j)\lambda$. Remember that $\mathbf{X}^j(x;\mathcal{D}^j) \in \mathbb{R}^{n\times KP}$ and represents the matrix of all of the $K$-Nearest Neighbor states from the previous $P$ laps close to the current state $x$. The product of this will create $\overline{x}$ which lives in $\mathbb{R}^n$ and is a linear combination of the $KP$ number of states closest to $x$ and the vector $\lambda$, with $\lambda$ linearly scaling the $KP$ number of states to create a target set. 

- The matrix $\mathbf{X}^j(x; \mathcal{D}^j)$ represents $K$ nearest-neighbor states from the previous $P$ laps that are closest to the current state $x$. This matrix is in $\mathbb{R}^{n \times KP}$, meaning it contains $KP$ state vectors, each in $\mathbb{R}^n$.
- To construct a target state $\overline{x} \in \mathbb{R}^n$, we must find a weight vector $\lambda \in \mathbb{R}^{KP}$ that satisfies:
    - Each element of $\lambda$ is between $0$ and $1$.
    - The sum of all elements in $\lambda$ equals $1$ (convex combination condition).
- Once such a $\lambda$ is found, we compute $\overline{x} = \mathbf{X}^j(x; \mathcal{D}^j) \lambda$. This means $\overline{x}$ is a weighted sum of the nearest-neighbor states, forming a convex hull of those states.
> [!NOTE] 
> We find the $KP$ number of nearest neighbors to the current state $x$. Then we multiply it by a weighted vector such that it follows the convex constraints laid about above. This will give us a region of possible vectors $\overline{x}$. These $\overline{x}$ states are feasible states that the system could potentially reach next from the current state $x$. We choose the state best to attain next, which I assume is some optimization problem such as finding the state closest to next point in the reference line that we're trying to follow. Then using the $f(x_k, u_k)$ function, we can tweak $u_k$ until we reach that state of $x_{k+1}$.

> [!NOTE] 
> Look into Quadratic Programming (QP) solvers. They're more efficient than gradient descent approach when considering a cost function is quadratic with linear constraints (quadratic functions with linear constraints are convex).

- The form of a quadratic programming problem is:  

$$
\min_{\lambda} \frac{1}{2}\lambda^\top Q \lambda + c^\top \lambda \\
\text{Q must be positive semi definite!}
$$
- But must have the constraints of the form:  

$$\begin{align*}
&A\lambda \le b, &\sum_i\lambda_i=1, &&0 \le \lambda_i \le 1
\end{align*}$$

### B. Local Terminal Cost-to-go Function
- The cost-to-go for any state $x^i_k$ (the $k\text{-th}$ state on lap $i$) is calculated through $T^i-k$ where $T^i$ is the number of states needed to complete lap $i$. $T^i-k$ represents the time left to complete the lap from the current state $x$.
- Now we're going to calculate the separate cost-to-go values for each state that's inside $\mathbf{X}^j(x; \mathcal{D}^j)$, the nearest neighbors found closest to current state $x$. We'll place the values inside a vector and name that $J^j_N(x;\mathcal{D}^j) \in \mathbb{R}^{KP}$. 
- After finding that cost-to-go vector, we can define a minimum cost-to-go function for any $\overline{x}\in\mathcal{X}^j_N(x;\mathcal{D}^j)$ as:  

$$\begin{align*}
    Q^j_N &(\overline{x}, x, \mathcal{D}^j) \\
        &= \min_{\lambda\in\mathbb{R}^{KP}} \space\space J^j_N(x; \mathcal{D}^j)^\top\lambda \\
        &\text{subject to }\space\space  0 \le \lambda \le 1, 1^\top\lambda=1, X^j(x, \mathcal{D})\lambda = \overline{x} \\
\end{align*}$$

- Notice how the same $\lambda$ for finding the optimal $\overline{x}$ is used in the cost function? This is a good way of using previously experienced costs, combine the costs in a way to find a minimum. 
- Once the $\lambda$ used to find the min is found, we can imply that the vectors used to create that list of costs can be linearly combined in the same way to create a feasible state $\overline{x}$. 
- When calculating that cost-to-go of that $\overline{x}$, it'll create that minimum cost-to-go calculated in $Q^j_N\text{.}$ 
- By only taking the $KP$ nearest states to the current state, we'll reduce computation loads significantly as the dataset grows because we'll only be taking the most relevant states. If we collect all KP states inside a matrix and we're trying to make a feasible state from them, we know that whatever $\overline{x}$ is created it'll span the columns of the matrix $\mathcal{X}^j_N$.

> [!NOTE]
> Terminal Cost-To-Go function is a cost function used at the final time step of an MPC horizon to encourage desirable long-term behavior. It helps account for the cost beyond the finite predicted horizon and helps ensure stability. It is typically a Lyapunov function, meaning it decreases over time, ensuring convergence. Often derived from a value function of an infinite-horizon optimal control problem. The value function for this paper would be equation (2). It helps to approximate the optimal cost-to-go beyond the finite LMPC horizon.

> [!NOTE]
> Terminal Target Set ($\mathcal{X}_N^j$) is a subset of states where the system is guaranteed to stabilize if the MPC reaches it at the final time step. The set $\mathcal{X}_f^j$​ is designed so that any state inside it can be driven to the final desired state using a feasible control law. Ensures recursive feasibility—meaning that the MPC can continue generating feasible solutions at each step. Usually, it is a control-invariant set, meaning once the system reaches it, it stays inside under the given control law.

> [!NOTE]
> A control-invariant set is a set of states where, for every state inside the set, there always exists a feasible control input that keeps the system inside the set. What does this mean for LMPC? If your vehicle reaches this set, you can always generate feasible control inputs that keep the vehicle stable and on track. The control-invariant set ensures that the MPC doesn't steer the vehicle into an unrecoverable position. 

- **Why Do We Need a Terminal Target Set?** *Since the LMPC only plans a few steps ahead (e.g., 10 or 20 states), we need a way to ensure that at the end of the planning horizon, the system is in a state that allows future motion without instability. The terminal target set forces the LMPC to steer toward a "safe" region that ensures the vehicle will remain feasible beyond the planning horizon.*

- **If we try to optimize for the lambda and get the optimal weighted vector in order to get to the optimal next state $\overline{x}$, how do we get the finite horizon that will give us a predicted trajectory?** *The answer is to recursively find the input optimal input $\overline{u}$ that is used to go from current state $x$ to optimal next state $\overline{x}$ using the $f(x_k, u_k) = x_{k+1}$. So if we have to find the trajectory 10 steps ahead, we'll have to repeatedly find $\overline{x}$ and plug in $\overline{u}$ and then use that $\overline{x}$ to find the next optimal state $\overline{x}_{k+1}$ until we're at $\overline{x}_{k+9}$, or $x_{k+10}$.*

### C. Local LMPC
- Using what we learned previously in the above sections, we can finally construct our LMPC control policy:  

$$\begin{align*}
    J^j_k (&x_k, u_{k-1}, \mathbf{\overline{z}}_k; \mathcal{D}^{j-1}) = \\ 
    
        &\min_{x,u,\lambda} \sum^{N-1}_{t=0}  \mathbb{1}_{\mathcal{F}}(x_t) + c_u|| u_t ||^2_2 + c_{\Delta u}|| u_t - u_{t-1} ||^2_2 \\ 
        
        &\quad\quad\quad\quad + J^{j-1}_N(\overline{x}_{k+N};\mathcal{D}^{j-1})^\top\lambda     \tag{4a} \\
    \text{subject to }& x_0=x_k, u_{-1}=u_{k-1} \tag{4b}  \\
    &x_{t+1} = A(\overline{z}_{k+t};\mathcal{D}^{j-1})x_t + B(\overline{z}_{k+t};\mathcal{D}^{j-1})u_t \\
    &\quad + C(\overline{z}_{k+t};\mathcal{D}^{j-1}), t \in \{0, ..., N-1\}, \tag{4c} \\
    &x_t \in \mathcal{X}, u_t \in \mathcal{U}, t \in \{0, ..., N \} \tag{4d} \\
    &\mathbf{X}^{j-1}(\overline{x}_{k+N};\mathcal{D}^{j-1})\lambda = x_N, \tag{4e} \\
    & 0 \le \lambda_i \le 1, \mathbb{1}^\top\lambda = 1 \tag{4f} \\ 
\end{align*}$$

where $\overline{z}_k = (\overline{x}_k, \overline{u}_k)$ and $\mathbf{\overline{z}}_k = \{\overline{z}_k, ..., \overline{z}_{K+N} \}$ is a state and input sequence which is used to form the local convex approximation. In practice, this is chosen as the solution to (4) from the previous time-step. Construction of the A, B, and C matrices in the ATV dynamics (4c) is the main contribution of this work and will be discussed in the next section. Note that (4) is a convex program which can be solved efficiently using existing solvers.


- $J^j_k (x_k, u_{k-1}, \mathbf{\overline{z}}_k; \mathcal{D}^{j-1})$ is read as the cost-to-go of state $x_k$ during iteration $j$, using state $x_k$, previous control input $u_{k-1}$, optimal set of states and inputs $\overline{\mathbf{z}}_k$ for the current finite horizon, and the previous iteration dataset $\mathcal{D}^{j-1}$.  
- Let's dissect this. The LMPC policy has 2 parts, the summation and the cost-to-go function for the last predicted state of the optimized finite horizon. We'll go through the summation first and then the cost-to-go for the last predicted state and why they're present.
- The summation has 5 parts to examine and we'll say what they represent. Here they are:
    - $\mathbb{1}_{\mathcal{F}}(x_t)$ : This is the indicator function that was defined in part (2). The purpose of this function is to keep adding 1 if state $x_t$ is before the lap is completed and add 0 if the state is has progressed past the finish line.
    - $c_u$ : This is a cost coefficent for penalizing the magnitude of control input $u_t$. We want this here because the last thing we want is to have a control input that has a high steering or high acceleration value. Either one will only lead the vehicle to violate the track boundaries or cause instability in the vehicle's controlling algorithm. In any case, having the vehicle accelerate or deccelerate too quickly will result in instability and cause the predefined vehicle dynamics model to fail. Same for steering, if the vehicle is going too fast and we steer too aggressively we'll make the vehicle skid and lose control.
    - $||u_t||^2_2$ : This is taking the squared L2 norm of the control input $u_t$. This will further penalize large magnitudes and encourage smaller parameters for inputting control parameters such as steering and acceleration.
    - $c_{\Delta u}$ : This is the cost coefficient for the change of control inputs from last state $u_{k-1}$ to current state $u_{k}$. The coefficient is used for penalizing if there's a major difference between sequential control inputs one after the other. 
    - $|| u_t - u_{t-1} ||^2_2$ : This is the squared L2 norm of the difference between the current control input $u_t$ and the last control input $u_{t-1}$. This will give us the difference of the 2 vectors, get the L2 norm of that, and then square it. This is useful because we want to the change in inputs to be as small as possible from one state to the next. The reason why this is needed is because we don't want the vehicle to jerk and respond irratically. If we had an $u_{t-1}$ that was steering the vehicle left and a $u_t$ that steers the vehicle right, that shift will violently shake the chassis and possibly drive the vehicle into a skidding state. To prevent situations like that, this term makes it less likely that the control inputs will cause the vehicle to become unstable. 
- Here's the second part of the function:
    - $J^{j-1}_N(\overline{x}_{k+N};\mathcal{D}^{j-1})$ is the cost-to-go of the last predicted state. This is saying that it will use the final predicted stated $\overline{x}_{k+N}$ where N is the number of steps ahead we want to predict from $k$. This is taking the state $\overline{x}_{k+N}$, finding the matrix $X$ that has the $KP$ states nearest to it using the dataset $\mathcal{D}^{j-1}$, and finding the cost-to-go vector of all of the states in $X$, or $J^{j-1}_N(\overline{x}_{k+N};\mathcal{D}^{j-1})$. 
    - $\lambda$ is the weighted vector multiplied to $J^{j-1}_N(\overline{x}_{k+N};\mathcal{D}^{j-1})$ to calculate the cost-to-go. 

## IV. LEARNED VEHICLE DYNAMICS MODEL
- In this section we'll get to see how the vehicle dynamics model is learned.
- We start with assuming how to model the true dynamics of the system and we do that with the following

$$
x^+ = f(x, u) + e(x, u), \tag{5}
$$

where $f$ is the function for the nominal dynamics in (1) and $e$ is the unknown modeling error which we would find out using data. 
- Next we define the prediction of the nominal model as $\hat{x}^+ = f(x, u)$ where x and u are the given state and input.

> [!NOTE]
> Linearizing a error dynamics is an essential topic to understand in optimization for robotics. Finding the true error dynamics of a system is often hard and almost impossible to find, so we try a local approximation through linearization. Remember that error is the difference between the current state $x$ and the reference state $\bar{x}$. 

- They try to linearize the error dynamics by using a reference state and input $\bar{z}$ as follows:

$$
x^+ - \hat{x}^+ = e(x, u) = A^ex + B^eu+C^e, \tag{6}
$$

where $A^e$ and $B^e$ are the jacobians of $e$ with respect to x and u evaluated at $\bar{z}$ and $C^e = e(\bar{x}, \bar{u}) - A^e\bar{x} - B^e\bar{u}$. 
- Think of the jacobians as sensitivity maps for the state and input. Remember that $A^e$ and $B^e$ are calculated using $\bar{z}$ from the pervious dataset $\mathcal{D}^{j-1}$ of the current iteration $\mathcal{D}^{j}$. For example, if you're using $x_k^j$ and $u_k^j$ for as $x$ and $u$ for (6), then we would use $x_k^{j-1}$ and $u_k^{j-1}$ for $\bar{x}$ and $\bar{u}$. 
- So rewriting the above equation, we would have something like this:

$$\begin{align*}
    e(x, u) &= A^ex + B^eu + C^e \\
    &= A^ex + B^eu + e(\bar{x}, \bar{u}) - A^e\bar{x} - B^e\bar{u} \\
    &= e(\bar{x}, \bar{u}) + A^ex - A^e\bar{x} + B^eu - B^e\bar{u} \\
    &= e(\bar{x}, \bar{u}) + A^e(x - \bar{x}) + B^e(u - \bar{u}) \\
\end{align*}$$

- Notice what we have at the end of the simplification: 
    - $e(\bar{x}, \bar{u})$ : The error of the optimal state $\bar{x}$ and optimal input $\bar{u}$ from last iteration $j-1$ at step $k$,
    - $A^e(x - \bar{x})$ : A mapping of the state error vector $(x - \bar{x})$ onto the sensitivity map or jacobian of $e$ evaluated $\bar{x}$ and $\bar{u}$. This term scales the affects of the difference between the nominal state and the current state on the system's dynamics.
    - $B^e(u - \bar{u})$ : A mapping of the input error vector $(u - \bar{u})$ onto the sensitivity map or jacobian of $e$ evaluated $\bar{x}$ and $\bar{u}$. This term scales the affects of the difference between the nominal input and the current input on the system's dynamics.

- At the end of this simplification, we get an error vector for the next state based on the current state and input. 

- We find the $M$ nearest neighbors in $\mathcal{D}^{j-1}$ to obtain $\mathbf{z} = \{z_1, ..., z_M\}$ of $\bar{z}$ and their corresponding state evolutions $\mathbf{x}^+ = \{x^+_1, ..., x^+_M\}$.
    - $\mathbf{x}^+ = \{x^+_1, ..., x^+_M\}$ : For every $x^+_{i}$, it is the state that came after the state inside the $z_i$. So an example is $z_1$ holds $(x_1, u_1)$ and the state that came after $x_1$ inside dataset $\mathcal{D}^{j-1}$ is $x^+_1$.

- We then use the state and input pairs in $\mathbf{z}$ to predict the states that they would have using our current model to find $\hat{\mathbf{x}}^+ = \{\hat{x}^+_1, ..., \hat{x}^+_M\}$.
    - $\hat{\mathbf{x}}^+ = \{\hat{x}^+_1, ..., \hat{x}^+_M\}$ : To get $\hat{x}^+_i$, we use our model function (1) to make the predictions using the state and input pair $z_i$ which could be replaced as $(x_i, u_i)$.

- So keep in mind that $\mathbf{z}$ has the $M$ nearest neighbors to $\bar{z}$ that are state and input pairs from dataset $\mathcal{D}^{j-1}$. Next, $\mathbf{x}^+$ holds the states that each $z$ in $\mathbf{z}$ progressed next unto in the previous iteration. Lastly, $\hat{\mathbf{x}}^+$ holds the predicted outputs using the state and input pairs $z$ found in $\mathbf{z}$.

- Now we're mainly interested in learning only the error dynamics associated with the velocity components of the state $x$.
- We'll define the residual error of the velocity components as below:

$$
A^e[11:13]x_m[1:3] + B^e[11]a_m + C^e[1] = v_{x, m}^+ - \hat{v}_{x, m}^+, \\
A^e[21:23]x_m[1:3] + B^e[22]\delta_m + C^e[2] = v_{y, m}^+ - \hat{v}_{y, m}^+, \\
A^e[31:33]x_m[1:3] + B^e[33]\delta_m + C^e[3] = \omega_{z, m}^+ - \hat{\omega}_{z, m}^+, \\
\tag{7}
$$

where $m \in \{1, ..., M\}$ and we use the notation $A[11 : 13]$ to index the first three elements of the first row of matrix $A$.

> [!NOTE]
> Confused on why $B^e$ is only being multiplied by $a_m$ once and $\delta_m$ other times? The reason is because of parameter definitions, system dynamics structure and dominant dependencies. The parameter definition of $v_x$ is longitudinal velocity which is the direction and magnitude of speed of the car parallel to the length of the body of the car. This would mean that steering would have negligible effects on the value of this parameter and the more dominant dependency would be acceleration. For the lateral velocity $v_y$ and yaw rate $\omega_z$, the first is the speed and direction perpendicular to the length of the body and the second is the orientation of the vehicle. These parameters are heavily affected by the steering vastly more than the acceleration, which is why it was deemed negligble when considering the error residual of that parameter and why acceleration is not being multiplied by $B^e$ on those lines.

- Let's breakdown each error dynamic line:
    - $v_{x, m}^+ - \hat{v}_{x, m}^+$ : Using the collection $\mathbf{x}$, we can use a specific state evolution that was recorded in the previous iteration. Then using the collection $\hat{\mathbf{x}}$, we can obtain the corresponding prediction that we used the current model to attain. Now if we find the difference between the recorded state evolution and current state prediction, we can see the difference between parameters. In this case it's the difference of the $v_{x, m}$. This expression is describing the residual error found between predictions of the previous and current model used in finding states when inputted state and input pairs.

    - $v_{y, m}^+ - \hat{v}_{y, m}^+$ : Using the collection $\mathbf{x}$, we can use a specific state evolution that was recorded in the previous iteration. Then using the collection $\hat{\mathbf{x}}$, we can obtain the corresponding prediction that we used the current model to attain. Now if we find the difference between the recorded state evolution and current state prediction, we can see the difference between parameters. In this case it's the difference of the $v_{y, m}$. This expression is describing the residual error found between predictions of the previous and current model used in finding states when inputted state and input pairs.
    
    - $\omega_{z, m}^+ - \hat{\omega}_{z, m}^+$ : Using the collection $\mathbf{x}$, we can use a specific state evolution that was recorded in the previous iteration. Then using the collection $\hat{\mathbf{x}}$, we can obtain the corresponding prediction that we used the current model to attain. Now if we find the difference between the recorded state evolution and current state prediction, we can see the difference between parameters. In this case it's the difference of the $\omega_{z, m}$. This expression is describing the residual error found between predictions of the previous and current model used in finding states when inputted state and input pairs.

- Let's use those 3 equations and create 3 regression vectors:  

$$
\Gamma_{v_x} = \begin{bmatrix}
           A^e[11:13] \\
           B^e[11] \\
           C^e[1]
         \end{bmatrix}, 

\Gamma_{v_y} = \begin{bmatrix}
           A^e[21:23] \\
           B^e[22] \\
           C^e[2]
         \end{bmatrix}, 

\Gamma_{\omega_z} = \begin{bmatrix}
           A^e[31:33] \\
           B^e[32] \\
           C^e[3]
         \end{bmatrix}.
$$

> [!NOTE]
> What are regression vectors? Regression vectors are often denoted as $\phi$, is a collection of variables (states, inputs, or past observations) that are used to estimate a system's response such as $y = \theta^T\phi + \epsilon$. In this equation, $y$ is the predicted response, $\theta$ is the set of parameters to be estimated, $\phi$ is the regressor vector containing relevant variables, and $\epsilon$ is the error or noise in the system. With our regressor vectors, notice how we only have the jacobian elements and none of the state or input parameters inside it? This is due to keeping our the $\theta$ outside to estimate the parameters later.

- We're close to solving the regression problem but we have to define a function that will weigh data point $m$ with greater importance than others. This is done using the Epanechnikov kernel function with bandwidth $h$:

$$
K(u) =
    \begin{cases}
      \frac{3}{4}(1 - u^2/h^2), &|u| < h \\
      0, & \text{otherwise}
    \end{cases}.
$$

- This function $K$ of $u$ will take in a value $u$ and will see if its magnitude is smaller than $h$. If it's smaller, then the function $K$ will output a number between $\frac{3}{4}$ and $0$. If magnitude of $u$ is greater than $h$, then the function will output $0$. The reason why this helps out so much is that if you were to place the L2 norm between 2 data points in here, then we can weigh the importance of that point. The closer the data point $z_m$ is to $\bar{z}$, the more important and relevant the data $z_m$ is which is why it should be weighed heavier than points that are far away from $\bar{z}$.

- To combine everything we've learned so far, we define a weighted least squares problem with $l = \{v_x, v_y, \omega_z\}$:  

$$
\Gamma^\star_l = \argmin_{\Gamma_l}\sum^M_{m=1} K(|| \bar{z} - z_m ||^2_Q) y^l_m(\Gamma_l) + \epsilon|| \Gamma_l ||^2_2, \tag{8}
$$

where $Q \succeq 0$ denotes the relative scaling between the variable, $y^l_m(\Gamma_l)$ are the $l^2$-norms of the residuals in (7), and $\epsilon>0$ is a regularization parameter. 

- Think of $(8)$ as a weighted ridge regression problem and rely on that knowledge to solve $\Gamma^\star_l$.

> [!NOTE]
> Why do we want a function to find the smallest $\Gamma^\star_l$ when we could solve the error dynamics equations for all of the nearest $M$ neighbors instead? Or vice-versa?? This is because of the 

- Lastly we can construct the matrices $A^e$, $B^e$, and $C^e$ from $\Gamma^\star_l$ as follows:

$$
A^e = \begin{bmatrix}
           \Gamma^\star_{v_x}[1:3] \\
           \Gamma^\star_{v_y}[1:3] \\
           \Gamma^\star_{\omega_z}[1:3] & \mathbf{0}_{6 \times 3} \\
           \mathbf{0}_{3 \times 3}
         \end{bmatrix}, 

B^e = \begin{bmatrix}
           \Gamma^\star_{v_x}[4] & 0 \\
           0 & \Gamma^\star_{v_y}[4] \\
           0 & \Gamma^\star_{\omega_z}[4] \\
           \mathbf{0}_{3 \times 2}
         \end{bmatrix}, \\

C^e = \begin{bmatrix} \Gamma^\star_{v_x}[5] & \Gamma^\star_{v_y}[5] & \Gamma^\star_{\omega_z}[5] & \mathbf{0}_{1 \times 3} \end{bmatrix}^\top.
$$

- Remember equation $(4c)$?  

$$
x_{t+1} = A(\overline{z}_{k+t};\mathcal{D}^{j-1})x_t + 
          B(\overline{z}_{k+t};\mathcal{D}^{j-1})u_t + 
          C(\overline{z}_{k+t} \mathcal{D}^{j-1}), t \in \{0, ..., N-1\}
$$

- We're going to make explain how to get $A(\overline{z}_{k+t};\mathcal{D}^{j-1})$, $B(\overline{z}_{k+t};\mathcal{D}^{j-1})$, and $C(\overline{z}_{k+t};\mathcal{D}^{j-1})$.
- Each term is the system dynamics mapping of how each parameter affects the next state. 
    - $A(\overline{z}_{k+t};\mathcal{D}^{j-1})$ term is a matrix that holds how each parameter in $x_t$ affects the state $x_{t+1}$.
    - $B(\overline{z}_{k+t};\mathcal{D}^{j-1})$ term is a matrix that holds how each parameter in $u_t$ affects the state $x_{t+1}$.
    - $C(\overline{z}_{k+t};\mathcal{D}^{j-1})$ term is a matrix that holds the true system dynamics of the environment, which is unaffected and unrelated to the current state $x_t$ and current input $u_t$.

$$
    A(\overline{z}_{k+t};\mathcal{D}^{j-1}) = A^f + A^e \\
    B(\overline{z}_{k+t};\mathcal{D}^{j-1}) = B^f + B^e \\
    C(\overline{z}_{k+t};\mathcal{D}^{j-1}) = C^f + C^e \\
$$

where $A^f$ and $B^f$ are the jacobians of $f$ w.r.t $x$ and $u$ evaluated at $\overline{z}$ and  
$C^f = f(\overline{x}, \overline{u}) - A^f\overline{x} - B^f\overline{u}$.

> [!NOTE]
> Why does $C^f$ substract the jacobian products A and B from nominal model's prediction? This is because we're trying to find the modeling sensitivity mapping of the environment. Because this part of the system dynamics focuses on how the environment itself affects the next state based on a specific part of the track and unaffected by the input $u$ or current state $x$, we want to subtract the contributions of the input parameters to the nominal prediction from the prediction so we can isolate the contribution of the environment. Mathematically it looks like this  
> $$\begin{align*}
    f(\overline{x}, \overline{u}) &= A^f\overline{x} + B^f\overline{u} + C^f \\
    C^f &= f(\overline{x}, \overline{u}) - A^f\overline{x} - B^f\overline{u} \\
        &= (A^f\overline{x} + B^f\overline{u} + C^f) - A^f\overline{x} - B^f\overline{u} \\
        &= C^f + A^f\overline{x} - A^f\overline{x} + B^f\overline{u} - B^f\overline{u} \\
        &= C^f
> \end{align*}$$

- 


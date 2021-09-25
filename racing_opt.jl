using JuMP
import Ipopt
import Plots

car = Model(Ipopt.Optimizer)
set_silent(car)

TRACK_WIDTH = 2 # Width of racetrack

x1_0 = 1    # Initial x position of player 1
y1_0 = 0    # Initial y position of player 1
v1_0 = 1    # Initial velocity of player 1
theta1_0 = 1    # Initial heading of player 1
a1_0 = 1    # Initial acceleration of player 1
phi1_0 = 1    # Initial steering angle of player 1

a_min1 = -6  # Maximum braking force for player 1
a_max1 = 6  # Maximum acceleration for player 1
phi_max1 = 30  # Maximum steering angle for player 1
L1 = 0.35 # Length of player 1's car
v_max1 = 5 # Max Velocity of player 1's car

T = 3         # Time Horizon
freq = 100    # Frequency
n = T * freq  # Total Time steps

@variables(car, begin
    Δt ≥ 0, (start = 1 / n) # Time step
    # State variables
    0 ≤ v1[1:n] ≤ vmax1           # Velocity
    -20 ≤ x1[1:n] ≤ 20    # X Position
    -20 ≤ y1[1:n] ≤ 20    # Y Position
    -20 ≤ theta1[1:n] ≤ 20    # Heading (I want to make this unbounded)

    # Control variables
    amin_1 ≤ a1[1:n] ≤ amax1    # Acceleration
    -phi_max1 ≤ phi1[1:n] ≤ phi_max1    # Acceleration
end)

function track_progression(x, y)
end

@objective(car, Max, track_progression(x[n], y[n]))

fix(x1[1], v1_0; force = true)
fix(y1[1], h1_0; force = true)
fix(v1[1], v1_0; force = true)
fix(theta1[1], theta1_0; force = true)
fix(a1[1], a1_0; force = true)
fix(phi1[1], ph1_0; force = true)

@NLexpressions(
    car,
    begin
        # Curvature(steering_angle) = tan(phi)*L
        kappa[j = 1:n], tan(phi1[j])*L1
        # Time of travel
        t_f, Δt * n
    end
)

function min_dist_to_center(x, y)

end

function calculate_lateral_acceleration(dv, dh)

end

for j in 2:n
    # Dynamics Constraints
    # Rectangular integration
    @NLconstraint(car, x1[j] == x1[j - 1] + Δt * v1[j - 1]*cos(theta1[j - 1))
    @NLconstraint(car, y1[j] == y1[j - 1] + Δt * v1[j - 1]*sin(theta1[j - 1))
    @NLconstraint(car, theta1[j] == theta1[j - 1] + Δt * v1[j - 1]*tan(phi[j - 1))
    @NLconstraint(car, v1[j] == v1[j - 1] + Δt * a1[j - 1])

    # Track Limits Constraints
    @NLconstraint(car, min_dist_to_center(x1[j], y1[j]) <= TRACK_WIDTH)

    # Lateral Acceleration Constraints
    @NLconstraint(car, calculate_lateral_acceleration(v1[j]-v1[j-1], h1[j]-h1[j-1]) <= max_cornering1)

end



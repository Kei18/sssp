using MRMP
using Random: seed!

# define models
config_init = [
    StateArm2(0.2, 0.2, 0.0, 0),
    StateArm2(0.5, 0.5, π/4, -π/2),
    StateArm2(0.8, 0.8, π, 0),
]
config_goal = [
    StateArm2(0.2, 0.2, π/2, -π/4),
    StateArm2(0.5, 0.5, -π/4, π/2),
    StateArm2(0.8, 0.8, -3π/4, -π),
]
obstacles = Vector{CircleObstacle2D}()
rads = fill(0.25, length(config_init))
MRMP.demo_get_initial_solution(config_init, config_goal, rads, obstacles; eps=0.2, goal_rad=0.05)

using MRMP
using Random: seed!

# define models
config_init =
    [StateLine2D(0.1, 0.1, 0), StateLine2D(0.1, 0.9, 0), StateLine2D(0.5, 0.1, π / 2)]
config_goal =
    [StateLine2D(0.9, 0.9, π), StateLine2D(0.9, 0.1, π), StateLine2D(0.5, 0.9, -π / 2)]
obstacles = [
    CircleObstacle2D(0.3, 0.5, 0.1),
    CircleObstacle2D(0.5, 0.6, 0.05),
    CircleObstacle2D(0.7, 0.3, 0.05),
]
rads = fill(0.2, length(config_init))
MRMP.demo_get_initial_solution(
    config_init, config_goal, rads, obstacles;
    eps=0.2, goal_rad = 0.05, other_params=Dict(:TIME_LIMIT => 30))

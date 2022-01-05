using MRMP
using Random: seed!

config_init = [StateCar(0.1, 0.1, 0), StateCar(0.1, 0.9, 0.0)]
config_goal = [StateCar(0.9, 0.9, π), StateCar(0.9, 0.1, π)]

obstacles = [
    CircleObstacle2D(0.3, 0.5, 0.1),
    CircleObstacle2D(0.5, 0.6, 0.05),
    CircleObstacle2D(0.7, 0.3, 0.05),
]
rads = fill(0.05, length(config_init))
MRMP.demo_get_initial_solution(
    config_init,
    config_goal,
    rads,
    obstacles;
    eps = 0.2,
    goal_rad = 0.05,
)

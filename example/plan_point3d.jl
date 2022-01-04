using MRMP
using Random: seed!

# define models
config_init = [StatePoint3D(0.1, 0.1, 0.1), StatePoint3D(0.9, 0.1, 0.9)]
config_goal = [StatePoint3D(0.9, 0.9, 0.9), StatePoint3D(0.1, 0.9, 0.1)]
obstacles = [
    CircleObstacle3D(0.3, 0.5, 0.5, 0.1),
    CircleObstacle3D(0.5, 0.6, 0.5, 0.05),
    CircleObstacle3D(0.7, 0.3, 0.5, 0.05),
]
rads = fill(0.1, length(config_init))
MRMP.demo_get_initial_solution(config_init, config_goal, rads, obstacles)

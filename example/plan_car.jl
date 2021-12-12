using MRMP
using Random: seed!

config_init = [
    StateCar(0.1, 0.1, 0),
    StateCar(0.1, 0.9, 0.0)
]
config_goal = [
    StateCar(0.9, 0.9, π),
    StateCar(0.9, 0.1, π)
]

obstacles = [
    CircleObstacle2D(0.3, 0.5, 0.1),
    CircleObstacle2D(0.5, 0.6, 0.05),
    CircleObstacle2D(0.7, 0.3, 0.05),
]
rads = fill(0.05, length(config_init))
eps = 0.2
goal_rad=0.05

# search
params=Dict(:MAX_ITER => 3, :MAX_LOOP_CNT => 20000)
seed!(0)
solution, roadmaps = simple_search(
    config_init, config_goal, obstacles, rads;
    eps=eps, goal_rad=goal_rad, sample_num_init=10, params=params)

# plot results
filename = "./local/car"
plot_res!(config_init, config_goal, obstacles, rads, roadmaps, solution; filename="$filename.pdf")
plot_anim!(config_init, config_goal, obstacles, rads, solution; filename="$filename.gif")

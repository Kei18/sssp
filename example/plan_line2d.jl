using MRMP
using Random: seed!

# define models
config_init = [
    StateLine2D(0.1, 0.1, 0),
    StateLine2D(0.1, 0.9, 0),
    StateLine2D(0.5, 0.1, π/2)
]
config_goal = [
    StateLine2D(0.9, 0.9, π),
    StateLine2D(0.9, 0.1, π),
    StateLine2D(0.5, 0.9, -π/2)
]
obstacles = [
    CircleObstacle2D(0.3, 0.5, 0.1),
    CircleObstacle2D(0.5, 0.6, 0.05),
    CircleObstacle2D(0.7, 0.3, 0.05),
]
rads = fill(0.2, length(config_init))
goal_rad=0.05

# search
params=Dict(:MAX_ITER => 3, :MAX_LOOP_CNT => 10000)
seed!(0)
solution, roadmaps = simple_search(config_init, config_goal, obstacles, rads;
                                   goal_rad=goal_rad, params=params)

# plot results
filename = "./local/line2d"
plot_res!(config_init, config_goal, obstacles, rads, roadmaps, solution; filename="$filename.pdf")
plot_anim!(config_init, config_goal, obstacles, rads, solution; filename="$filename.gif")

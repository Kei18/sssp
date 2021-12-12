using MRMP
using Random: seed!

# define models
config_init = [StatePoint2D(0.1, 0.1), StatePoint2D(0.1, 0.9)]
config_goal = [StatePoint2D(0.9, 0.9), StatePoint2D(0.9, 0.1)]
obstacles = [
    CircleObstacle2D(0.3, 0.5, 0.1),
    CircleObstacle2D(0.5, 0.6, 0.05),
    CircleObstacle2D(0.7, 0.3, 0.05),
]
rads = fill(0.1, length(config_init))

# search
params=Dict(:MAX_ITER => 3, :MAX_LOOP_CNT => 1000)
seed!(0)
solution, roadmaps = simple_search(config_init, config_goal, obstacles, rads; params=params)

# plot results
filename = "./local/point2d"
plot_res!(config_init, config_goal, obstacles, rads, roadmaps, solution;
          filename="$filename.pdf")
plot_anim!(config_init, config_goal, obstacles, rads, solution; filename="$filename.gif")

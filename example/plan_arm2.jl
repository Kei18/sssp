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
eps = 0.2

# search
params=Dict(:MAX_ITER => 3, :MAX_LOOP_CNT => 10000)
seed!(0)
solution, roadmaps = simple_search(config_init, config_goal, obstacles, rads;
                                   eps=eps, params=params)

# plot results
filename = "./local/arm2"
plot_res!(config_init, config_goal, obstacles, rads, roadmaps, solution; filename="$filename.pdf")
plot_anim!(config_init, config_goal, obstacles, rads, solution; filename="$filename.gif")

using MRMP
using Random: seed!

# define models
config_init = [
    StateArm2(0.2, 0.2, 0.0, 0),
    StateArm2(0.5, 0.5, π/4, -π/2),
    StateArm2(0.8, 0.8, π, 0),
    StateArm2(0.2, 0.8, -π/2, π/2)
]
config_goal = [
    StateArm2(0.2, 0.2, π/2, -π/4),
    StateArm2(0.5, 0.5, -π/4, π/2),
    StateArm2(0.8, 0.8, -3π/4, -π),
    StateArm2(0.2, 0.8, -2π/3, π/4)
]
obstacles = Vector{CircleObstacle2D}()
rads = fill(0.25, length(config_init))
eps = 0.4

# search
params=Dict(:MAX_ITER => 3, :MAX_LOOP_CNT => 10000)
seed!(0)
S, V, VISITED = simple_search(config_init, config_goal, obstacles, rads;
                              eps=eps, params=params)

# plot results
filename = "./local/arm2"
plot_res!(config_init, config_goal, obstacles, rads, V, S, VISITED; filename="$filename.pdf")
plot_anim!(config_init, config_goal, obstacles, rads, S, VISITED;
           filename="$filename.gif", fps=30)

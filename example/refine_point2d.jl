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
eps = 0.2

# search
params=Dict(:MAX_ITER => 3, :MAX_LOOP_CNT => 100000)

q = config_init[1]
connect = gen_connect(q, rads, obstacles, eps)
collide = gen_collide(q, rads)
check_goal = gen_check_goal(config_goal)
h_func = gen_h_func(config_goal)
g_func = gen_g_func(greedy=true)
random_walk = gen_random_walk(q, eps)
get_sample_nums = gen_get_sample_nums(3)
seed!(0)

solution_before, roadmaps = search!(
    config_init, config_goal, connect, collide, check_goal,
    h_func, g_func, random_walk, get_sample_nums; params...)

println("refinement start")
solution_after, roadmaps = search!(
    config_init, config_goal, connect, collide, check_goal,
    h_func, g_func, random_walk, get_sample_nums;
    current_solution=solution_before,
    roadmaps=roadmaps,
    params...)

# plot results
filename = "./local/refine_point2d"
plot_anim!(config_init, config_goal, obstacles, rads, solution_before;
           filename="$filename"*"_before.gif")
plot_anim!(config_init, config_goal, obstacles, rads, solution_after;
           filename="$filename"*"_after.gif")

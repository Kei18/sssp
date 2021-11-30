using MRMP
using Random: seed!

# define models
config_init = [StateLine2D(0.1, 0.1, 0), StateLine2D(0.1, 0.9, 0), StateLine2D(0.5, 0.1, π/2)]
config_goal = [StateLine2D(0.9, 0.9, π), StateLine2D(0.9, 0.1, π), StateLine2D(0.5, 0.9, -π/2)]
obstacles = [
    CircleObstacle2D(0.3, 0.5, 0.1),
    CircleObstacle2D(0.5, 0.6, 0.05),
    CircleObstacle2D(0.7, 0.3, 0.05),
]
rads = fill(0.2, length(config_init))
eps = 0.1
connect = gen_connect_line(rads, obstacles, eps)
collide = gen_collide_line(rads)
check_goal = gen_check_goal(config_goal, goal_rad=0.05)
h_func = gen_h_func(config_goal)
g_func = gen_g_func(config_init, greedy=true)
random_walk = gen_random_walk(eps)
get_sample_nums = gen_get_sample_nums(3)

# search
seed!(0)
S, V, VISITED = search(
    config_init,
    config_goal,
    connect,
    collide,
    check_goal,
    h_func,
    g_func,
    random_walk,
    get_sample_nums;
    MAX_ITER=3,
    VERBOSE_LOOP_CNT=10,
    MAX_LOOP_CNT=1000,
)
directory = "./local/"
if !isdir(directory); mkpath(directory); end
plot_res!(config_init, config_goal, obstacles, V, S, VISITED, filename=directory*"/line2d.pdf")
if S != nothing
    plot_anim!(config_init, config_goal, obstacles, S, VISITED, rads,
               filename=directory*"/line2d.gif")
end
nothing

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
eps = 0.1
connect = gen_connect_point(rads, obstacles, eps)
collide = gen_collide_point(rads)
check_goal = gen_check_goal(config_goal)
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
    MAX_ITER=1,
    VERBOSE_LOOP_CNT=1,
    MAX_LOOP_CNT=10000,
)
directory = "./local/"
if !isdir(directory); mkpath(directory); end
plot_res!(config_init, config_goal, obstacles, V, S, VISITED, filename=directory*"/point3d.pdf")
if S != nothing
    plot_anim!(config_init, config_goal, obstacles, S, VISITED, rads,
               filename=directory*"/point3d.gif")
end
nothing

using MRMP
using Random: seed!

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
connect = gen_connect_arm2(rads, obstacles, eps)
collide = gen_collide_arm2(rads)
check_goal = gen_check_goal(config_goal, goal_rad=0.0)
h_func = gen_h_func(config_goal)
g_func = gen_g_func(config_init, greedy=true)
random_walk = gen_random_walk(eps)
get_sample_nums = gen_get_sample_nums(5)

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
    VERBOSE_LOOP_CNT=100,
    MAX_LOOP_CNT=10000,
)
directory = "./local/"
if !isdir(directory); mkpath(directory); end
plot_res!(config_init, config_goal, obstacles, V, S, VISITED; rads=rads, filename=directory*"/arm2.pdf")
if S != nothing
    plot_anim!(config_init, config_goal, obstacles, S, VISITED, rads,
               filename=directory*"/arm2.gif")
end
nothing

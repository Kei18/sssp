using MRMP
using Random: seed!

config_init = [StateCar(0.1, 0.1, 0), StateCar(0.1, 0.9, 0.0)]
config_goal = [StateCar(0.9, 0.9, π), StateCar(0.9, 0.1, π)]

obstacles = [
    CircleObstacle2D(0.3, 0.5, 0.1),
    CircleObstacle2D(0.5, 0.6, 0.05),
    CircleObstacle2D(0.7, 0.3, 0.05),
]
omega_max = 4π
rads = fill(0.05, length(config_init))
eps = 0.2
connect = gen_connect_car(rads, obstacles, omega_max, eps)
collide = gen_collide_car(rads, omega_max)
check_goal = gen_check_goal(config_goal, goal_rad=0.05)
h_func = gen_h_func(config_goal)
g_func = gen_g_func(config_init, greedy=true)
random_walk = gen_random_walk(eps, omega_max)
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
    MAX_LOOP_CNT=100000,
)
directory = "./local/"
if !isdir(directory); mkpath(directory); end
plot_res!(config_init, config_goal, obstacles, V, S, VISITED;
          rads=rads, filename=directory*"/car.pdf")
if S != nothing
    plot_anim!(config_init, config_goal, obstacles, S, VISITED, rads,
               filename=directory*"/car.gif")
end
nothing

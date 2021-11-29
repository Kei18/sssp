using MRMP: StatePoint2D, CircleObstacle2D, search, gen_collide_point2d, gen_connect_point2d, dist, plot_anim!
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
eps = 0.1
connect = gen_connect_point2d(rads, obstacles, eps)
collide = gen_collide_point2d(rads)
check_goal = (Q) -> all([dist(v.q, q) <= 0 for (v, q) in zip(Q, config_goal)])
h_func = (Q) -> sum([dist(v.q, config_goal[i]) for (i, v) in enumerate(Q)])
g_func = (Q_from, Q_to) -> 0
random_walk = (q) -> begin
    theta = rand() * 2π
    rad = rand() * eps
    return StatePoint2D(cos(theta) * rad + q.x, sin(theta) * rad + q.y)
end
get_sample_nums = (k::Int64) -> k + 2

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
    MAX_ITER=2,
    VERBOSE_LOOP_CNT=10,
    MAX_LOOP_CNT=1000,
)
@test S != nothing
directory = "./local/"
if !isdir(directory); mkpath(directory); end
plot_anim!(config_init, config_goal, obstacles, S, VISITED, rads,
           filename=directory*"/point2d.gif")

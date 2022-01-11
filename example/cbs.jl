using MRMP
import Random: seed!

# define models
config_init = [StatePoint2D(0.1, 0.1), StatePoint2D(0.1, 0.9)]
config_goal = [StatePoint2D(0.9, 0.9), StatePoint2D(0.9, 0.1)]
rads = fill(0.1, length(config_init))
obstacles = [
    CircleObstacle2D(0.3, 0.5, 0.1),
    CircleObstacle2D(0.5, 0.6, 0.05),
    CircleObstacle2D(0.7, 0.3, 0.05),
]

q = config_init[1]
connect = gen_connect(q, rads, obstacles)
collide = gen_collide(q, rads)
check_goal = MRMP.gen_check_goal(config_goal)
g_func = MRMP.gen_g_func(stay_penalty = 0.1)

seed!(1)
solution, roadmaps = MRMP.conflict_based_search(
    config_init,
    config_goal,
    connect,
    collide,
    check_goal,
    g_func;
    num_vertices = 50,
    roadmaps_growing_rate = 1.1,
    VERBOSE = 2,
    max_makespan = 10,
    collision_weight = 0.5,
)

# plot results
filename = "./local/" * string(MRMP.now())
@async plot_res!(
    config_init,
    config_goal,
    obstacles,
    rads,
    roadmaps,
    solution;
    filename = "$filename/roadmap.pdf",
)
@async plot_anim!(
    config_init,
    config_goal,
    obstacles,
    rads,
    solution;
    filename = "$filename/traj.gif",
)
nothing

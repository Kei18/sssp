"""
# MRMP: Multi-Robot Motion Planning Library

## minimum examples

```jl
# 1. generate instance
ins = MRMP.gen_random_instance_StatePoint2D(;
    N = 5,
    rad = 0.1,
    num_obs = 3,
    rad_obs = 0.1,
)
config_init, config_goal, obstacles, ins_params... = ins         # example of ins_params: radius, base positions of arms

# 2. define functions
connect = gen_connect(config_init[1], obstacles, ins_params...)  # connection checker
collide = gen_collide(config_init[1], ins_params...)             # collision checker
check_goal = gen_check_goal(config_goal)                         # goal judge

# 3. solve
@time solution, roadmaps = MRMP.Solvers.planner3(
    config_init,
    config_goal,
    connect,
    collide,
    check_goal;
    TIME_LIMIT = 10,
)
validate(config_init, connect, collide, check_goal, solution)    # check validity of solution

# 4. refine
(TPG, solution, cost) = smoothing(solution, collide, connect)
println(cost)

# 5. visualize
plot_anim!(config_init, config_goal, obstacles, ins_params...; solution=solution, interpolate_depth=2)
```

## model definition

A model X (in `./models`) is defined as follows

- `StateX <: AbsState` defines configuration space
- `get_mid_status(q1::StateX, q2::StateX)` returns intermediate state
- `dist(q1::StateX, q2::StateX)` returns distance between two states
- `gen_connect(q::StateX,  obstacles, other_params...)` generates connect check function
- `gen_collide(q::StateX, other_params...)` generates inter-agent collision check function
- `gen_uniform_sampling(q::StateX)` generates uniform sampling function for StateX
- `plot_motion!(q_from::StateX, q_to::StateX, other_params...)` plots motion of one agent
- `plot_start_goal!(q_init::StateX, q_goal::StateX, other_params...)` plots start and goal statues
- `plot_agent!(q::StateX, other_params...)` plots an agent
- `gen_random_instance_StateX(; params...)` generates random instance, see `./utils.jl`

"""
module MRMP

import DataStructures: PriorityQueue, enqueue!, dequeue!
import Printf: @printf, @sprintf
import Base: @kwdef
import Random: randperm
using LinearAlgebra: norm, dot, normalize

# parameters
const STEP_DIST = 0.01
const SAFETY_DIST_LINE = 0.01
const DUBINS_TURN_RADIUS = 1.0

# state (i.e., sampled point), defined in ./models
abstract type AbsState end

# vertex in roadmaps
mutable struct Node{State<:AbsState}
    q::State   # corresponding state
    id::Int64  # unique id
    neighbors::Vector{Int64}  # indexes
end

include("obstacle.jl")
include("smoother.jl")
include("viz.jl")
include("utils.jl")

# models
include("models/point.jl")
include("models/point2d.jl")
include("models/point3d.jl")
include("models/line2d.jl")
include("models/arm22.jl")
include("models/arm33.jl")
include("models/snake2d.jl")
include("models/dubins2d.jl")

# solvers
include("solvers/solvers.jl")

export gen_connect, gen_collide, gen_check_goal
export plot_res!, plot_anim!, plot_instance!
export smoothing, get_solution_cost, get_tpg_cost, is_valid_instance, validate
export gen_random_instance_StateArm22,
    gen_random_instance_StateArm33,
    gen_random_instance_StateDubins,
    gen_random_instance_StateLine2D,
    gen_random_instance_StatePoint2D,
    gen_random_instance_StatePoint3D,
    gen_random_instance_StateSnake2D

using .Solvers

end

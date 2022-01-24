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
abstract type StatePoint <: AbsState end

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
export smoothing
export print_instance, is_valid_instance

using .Solvers

end

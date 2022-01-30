"""
# solvers for multi-robot motion planning

## included solvers
- PRM: probabilistic roadmap
- CBS: PRM + conflict-based search
- PP: PRM + prioritized planning
- RRT: rapidly-exploring random tree
- RRT-connect
- plannerX
"""
module Solvers

using MRMP
import ..MRMP: AbsState, Node, dist, now, elapsed_sec, gen_uniform_sampling, get_mid_status
import Printf: @sprintf, @printf
import Base: @kwdef
import DataStructures: PriorityQueue, enqueue!, dequeue!
import Random: randperm

include("utils.jl")
include("prm.jl")
include("rrt.jl")
include("rrt_connect.jl")
include("pp.jl")
include("cbs.jl")
include("plannerx.jl")

using .LibPP, .LibCBS, .LibRRT, .LibRRT_connect, .LibPlannerX
export PRM_direct, PRMs, PRMs!, RRT, RRT_connect, PP, CBS, planner3
end

"""
# solvers for multi-robot motion planning

## included solvers
- PRM: probabilistic roadmap
- CBS: PRM + conflict-based search
- PP: PRM + prioritized planning
- RRT: rapidly-exploring random tree
- RRT-connect
- SSSP: simultaneous sampling-and-search planning
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
include("sssp.jl")

using .LibPP, .LibCBS, .LibRRT, .LibRRT_connect, .LibSSSP
export PRM_direct, PRMs, PRMs!, RRT, RRT_connect, PP, CBS, SSSP
end

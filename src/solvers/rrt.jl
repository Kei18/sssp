"""Implementation of RRT

ref:
- LaValle, S. M. (1998). Rapidly-exploring random trees: A new tool for path planning.
"""
module LibRRT
export RRT

import Printf: @sprintf, @printf
import ...MRMP: AbsState, Node, now, elapsed_sec, gen_uniform_sampling, get_mid_status, dist

"""
    RRT(
        config_init::Vector{State},
        config_goal::Vector{State},
        connect::Function,
        collide::Function,
        check_goal::Function;
        steering_depth::Int64 = 4,                  # steering parameter
        epsilon::Union{Float64,Nothing} = 0.2,      # \epsilon neighbor
        TIME_LIMIT::Union{Nothing,Real} = nothing,
        VERBOSE::Int64 = 0,
    )::Tuple{
        Union{Nothing,Vector{Vector{Node{State}}}},  # solution
        Vector{Vector{Node{State}}},  # roadmaps
    } where {State<:AbsState}

implementation of RRT
"""

function RRT(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function;
    steering_depth::Int64 = 4,
    epsilon::Union{Float64,Nothing} = 0.2,
    TIME_LIMIT::Union{Nothing,Real} = nothing,
    VERBOSE::Int64 = 0,
)::Tuple{
    Union{Nothing,Vector{Vector{Node{State}}}},  # solution
    Vector{Vector{Node{State}}},  # roadmaps
} where {State<:AbsState}

    # utilities for timeout
    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    # number of agents
    N = length(config_init)

    # define sampler
    _sampler = gen_uniform_sampling(config_init[1])
    sampler() = map(i -> _sampler(), 1:N)

    # re-define connect function
    connect_C(C_from::Vector{State}, C_to::Vector{State})::Bool = begin
        # check distance
        !(isnothing(epsilon) || dist(C_from, C_to) <= epsilon) && return false

        # check collision with obstacles
        any(i -> !connect(C_from[i], C_to[i], i), 1:N) && return false

        # check inter-agent collisions
        for i = 1:N, j = i+1:N
            collide(C_from[i], C_to[i], C_from[j], C_to[j], i, j) && return false
        end

        return true
    end

    # store all samples
    V = [config_init]
    # parents index
    P = [-1]

    # special case
    if connect_C(config_init, config_goal)
        return (
            [
                map(q -> Node{State}(q, 1, []), config_init),
                map(q -> Node{State}(q, 1, []), config_goal),
            ],
            get_roadmaps(V, P),
        )
    end

    iter = 0
    while !timeover()
        iter += 1

        VERBOSE > 1 &&
            iter % 100 == 0 &&
            @printf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)

        # new sample
        C_h = sampler()
        # find parent
        ind_near = findmin(C -> dist(C, C_h), V)[end]
        C_near = V[ind_near]

        # steering
        C_l = C_near  # safe sample
        if !connect_C(C_l, C_h)
            for _ = 1:steering_depth
                C = map(e -> get_mid_status(e...), zip(C_l, C_h))
                if connect_C(C_near, C)
                    C_l = C
                else
                    C_h = C
                end
            end
            C_h = C_l
        end

        # update graph
        push!(V, C_h)
        push!(P, ind_near)

        # try to connect goal
        if connect_C(C_h, config_goal)
            push!(V, config_goal)
            push!(P, length(V) - 1)
            C_h = config_goal
        end

        # goal connection check
        if check_goal(C_h)
            VERBOSE > 1 && @printf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)
            VERBOSE > 0 && @info @sprintf("\t%6.4f sec: found solution\n", elapsed())
            return (backtrack(V, P), get_roadmaps(V, P))
        end

    end

    VERBOSE > 0 && @info @sprintf("\t%6.4f sec: failed to find solution\n", elapsed())
    return (nothing, get_roadmaps(V, P))
end

function backtrack(
    V::Vector{Vector{State}},  # tree rooted config_init
    P::Vector{Int64},  # parent indexes
)::Vector{Vector{Node{State}}} where {State<:AbsState}

    solution = Vector{Vector{Node{State}}}()
    ind = length(V)
    while ind != -1
        pushfirst!(solution, map(q -> Node{State}(q, ind, []), V[ind]))
        ind = P[ind]
    end
    solution
end

function get_roadmaps(
    V::Vector{Vector{State}},  # tree rooted config_init
    P::Vector{Int64},  # parent indexes
)::Vector{Vector{Node{State}}} where {State<:AbsState}
    """create roadmaps for visualization"""

    roadmaps = map(i -> Vector{Node{State}}(), 1:length(V[1]))
    for (k, C) in enumerate(V)
        for (i, q) in enumerate(C)
            push!(roadmaps[i], Node{State}(q, k, []))
            P[k] > 0 && push!(roadmaps[i][P[k]].neighbors, k)
        end
    end
    roadmaps
end

end

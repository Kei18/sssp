module LibRRT_connect
export RRT_connect

import Printf: @sprintf, @printf
import ...MRMP: AbsState, Node, now, elapsed_sec, gen_uniform_sampling, get_mid_status, dist

function RRT_connect(
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
    V1 = [config_init]
    V2 = [config_goal]
    # parents index
    P1 = [-1]
    P2 = [-1]

    V = V1
    P = P1

    # special case
    if connect_C(config_init, config_goal)
        return (
            [
                map(q -> Node{State}(q, 1, []), config_init),
                map(q -> Node{State}(q, 1, []), config_goal),
            ],
            get_roadmaps(V1, P1, V2, P2),
        )
    end

    # main loop
    iter = 0
    from_start = false
    while !timeover()
        iter += 1
        from_start = !from_start

        VERBOSE > 1 &&
            iter % 100 == 0 &&
            @printf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)

        C_new = extend!(connect_C, sampler(), V1, P1, from_start, steering_depth)[end]
        if extend!(connect_C, C_new, V2, P2, !from_start, steering_depth)[1] == :reached
            # solved
            VERBOSE > 1 && @info @sprintf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)
            VERBOSE > 0 && @info @sprintf("\t%6.4f sec: found solution\n", elapsed())
            return (
                from_start ? (backtrack(V1, P1, V2, P2), get_roadmaps(V1, P1, V2, P2)) :
                (backtrack(V2, P2, V1, P1), get_roadmaps(V2, P2, V1, P1))
            )
        end

        # swap tree
        V1, V2 = V2, V1
        P1, P2 = P2, P1
    end

    VERBOSE > 0 && @info @sprintf("\t%6.4f sec: failed to find solution\n", elapsed())

    return (
        nothing,
        from_start ? get_roadmaps(V2, P2, V1, P1) : get_roadmaps(V1, P1, V2, P2),
    )
end


# define expand function
function extend!(
    connect::Function,
    C_rand::Vector{State},  # sampled point
    V::Vector{Vector{State}},  # vertices
    P::Vector{Int64},  # parent indexes
    from_start::Bool,  # true -> tree rooted C_init, false -> tree rooted C_goal
    steering_depth::Int64,
)::Tuple{Symbol,Vector{State}} where {State<:AbsState}

    # find nearest existing vertex
    ind_near = (
        from_start ? findmin(C -> dist(C, C_rand), V)[end] :
        findmin(C -> dist(C_rand, C), V)[end]
    )
    C_near = V[ind_near]
    flg = :reached

    C_l = C_near  # safe sample
    C_h = C_rand  # probably unsafe sample -> eventually safe

    # steering, binary search
    if from_start ? !connect(C_near, C_h) : !connect(C_h, C_near)
        flg = :advanced
        for _ = 1:steering_depth
            C = map(e -> get_mid_status(e...), from_start ? zip(C_l, C_h) : zip(C_h, C_l))
            if from_start ? connect(C_near, C) : connect(C, C_near)
                C_l = C
            else
                C_h = C
            end
        end
        C_h = C_l
    end
    push!(V, C_h)
    push!(P, ind_near)

    return (flg, C_h)
end

function backtrack(
    V1::Vector{Vector{State}},  # tree rooted config_init
    P1::Vector{Int64},
    V2::Vector{Vector{State}},  # tree rooted config_goal
    P2::Vector{Int64},
)::Vector{Vector{Node{State}}} where {State<:AbsState}
    """create solution"""

    solution = Vector{Vector{Node{State}}}()
    ind = length(V1)
    while ind != -1
        pushfirst!(solution, map(q -> Node{State}(q, ind, []), V1[ind]))
        ind = P1[ind]
    end
    solution = solution[1:end-1]
    ind = length(V2)
    while ind != -1
        push!(solution, map(q -> Node{State}(q, ind, []), V2[ind]))
        ind = P2[ind]
    end
    solution
end

function get_roadmaps(
    V1::Vector{Vector{State}},  # tree rooted config_init
    P1::Vector{Int64},
    V2::Vector{Vector{State}},  # tree rooted config_goal
    P2::Vector{Int64},
)::Vector{Vector{Node{State}}} where {State<:AbsState}
    """create roadmaps for visualization"""

    roadmaps = map(i -> Vector{Node{State}}(), 1:length(V1[1]))
    for (k, C) in enumerate(V1)
        for (i, q) in enumerate(C)
            push!(roadmaps[i], Node{State}(q, k, []))
            P1[k] > 0 && push!(roadmaps[i][P1[k]].neighbors, k)
        end
    end
    K = length(V1)
    for (k, C) in enumerate(V2)
        for (i, q) in enumerate(C)
            push!(roadmaps[i], Node{State}(q, k, P2[k] > 0 ? [P2[k] + K] : []))
        end
    end
    roadmaps
end

end

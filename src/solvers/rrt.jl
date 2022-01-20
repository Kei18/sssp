function RRT(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function;
    steering_depth::Int64 = 4,
    TIME_LIMIT::Union{Nothing,Real} = nothing,
    VERBOSE::Int64 = 0,
)::Tuple{
    Union{Nothing,Vector{Vector{Node{State}}}},  # solution
    Vector{Vector{Node{State}}},  # roadmaps
} where {State<:AbsState}

    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    N = length(config_init)

    # define utilities
    _sampler = MRMP.gen_uniform_sampling(config_init[1])
    sampler() = map(i -> _sampler(), 1:N)
    connect_C(C_from::Vector{State}, C_to::Vector{State}) = begin
        all(i -> connect(C_from[i], C_to[i], i), 1:N) && begin
            for i = 1:N, j = i+1:N
                if collide(C_from[i], C_to[i], C_from[j], C_to[j], i, j)
                    return false
                end
            end
            return true
        end
    end

    get_roadmaps() = begin
        roadmaps = map(i -> Vector{Node{State}}(), 1:N)
        for (k, C) in enumerate(V)
            for (i, q) in enumerate(C)
                push!(roadmaps[i], Node{State}(q, k, k == 1 ? [] : [P[k]]))
            end
        end
        roadmaps
    end

    backtrack() = begin
        solution = Vector{Vector{Node{State}}}()
        ind = length(V)
        while !(ind == -1)
            pushfirst!(solution, map(q -> Node{State}(q, ind, []), V[ind]))
            ind = P[ind]
        end
        solution
    end

    # store all samples
    V = [ config_init ]
    # parents index
    P = [ -1 ]

    iter = 0
    while !timeover()
        iter += 1

        VERBOSE > 1 && iter % 100 == 0 && @printf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)

        # new sample
        C_h = iter != 1 ? sampler() : config_goal
        # find parent
        _, ind_near = findmin(C -> dist(C, C_h), V)
        C_near = V[ind_near]

        # steering
        C_l = C_near  # safe sample
        if !connect_C(C_l, C_h)
            for _ = 1:steering_depth
                C = map(e -> MRMP.get_mid_status(e...), zip(C_l, C_h))
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
            push!(P, length(V)-1)
            C_h = config_goal
        end

        # goal connection check
        if check_goal(C_h)
            if VERBOSE > 0
                VERBOSE > 1 && @printf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)
                @printf("\t%6.4f sec: found solution\n", elapsed())
            end
            return (backtrack(), get_roadmaps())
        end

    end

    if VERBOSE > 0
        @printf("\t%6.4f sec: failed to find solution\n", elapsed())
    end
    return (nothing, get_roadmaps())
end

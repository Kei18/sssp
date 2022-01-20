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

    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    N = length(config_init)

    # define utilities
    _sampler = MRMP.gen_uniform_sampling(config_init[1])
    sampler() = map(i -> _sampler(), 1:N)
    connect_C(C_from::Vector{State}, C_to::Vector{State}) = begin
        (isnothing(epsilon) || dist(C_from, C_to) <= epsilon) &&
        all(i -> connect(C_from[i], C_to[i], i), 1:N) &&
        begin
            for i = 1:N, j = i+1:N
                if collide(C_from[i], C_to[i], C_from[j], C_to[j], i, j)
                    return false
                end
            end
            return true
        end
    end

    extend!(C_rand, V, P) = begin
        ind_near = findmin(C -> dist(C, C_rand), V)[end]
        C_near = V[ind_near]
        flg = :reached

        # steering
        C_l = C_near  # safe sample
        C_h = C_rand
        if !connect_C(C_l, C_h)
            flg = :advanced
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

        return (flg, C_h)
    end

    get_roadmaps() = begin
        roadmaps = map(i -> Vector{Node{State}}(), 1:N)
        for (k, C) in enumerate(V1)
            for (i, q) in enumerate(C)
                push!(roadmaps[i], Node{State}(q, k, P1[k] == -1 ? [] : [P1[k]]))
            end
        end
        K = length(V1)
        for (k, C) in enumerate(V2)
            for (i, q) in enumerate(C)
                push!(roadmaps[i], Node{State}(q, k, P2[k] == -1 ? [] : [P2[k] + K]))
            end
        end
        roadmaps
    end

    backtrack!() = begin
        # create solution
        solution = Vector{Vector{Node{State}}}()

        # fix tree
        if V1[1] == config_goal
            V1, V2 = V2, V1
            P1, P2 = P2, P1
        end

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

        return solution
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
            get_roadmaps(),
        )
    end

    # main loop
    iter = 0
    while !timeover()
        iter += 1

        VERBOSE > 1 &&
            iter % 100 == 0 &&
            @printf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)

        C_new = extend!(sampler(), V1, P1)[end]
        if extend!(C_new, V2, P2)[1] == :reached
            VERBOSE > 1 && @printf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)
            VERBOSE > 0 && @printf("\t%6.4f sec: found solution\n", elapsed())
            return (backtrack!(), get_roadmaps())
        end

        # swap tree
        V1, V2 = V2, V1
        P1, P2 = P2, P1
    end

    VERBOSE > 0 && @printf("\t%6.4f sec: failed to find solution\n", elapsed())

    return (nothing, get_roadmaps())
end

function PP(
    roadmaps::Vector{Vector{Node{State}}},
    collide::Function,
    check_goal::Function,
    g_func::Function;
    max_makespan::Int64 = 20,
    order_randomize::Bool = true,
    TIME_LIMIT::Union{Nothing,Real} = nothing,
    VERBOSE::Int64 = 0,
)::Tuple{
    Union{Nothing,Vector{Vector{Node{State}}}},  # solution
    Vector{Vector{Node{State}}},  # roadmaps
} where {State<:AbsState}

    # timeout
    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    N = length(roadmaps)
    distance_tables = get_distance_tables(roadmaps, g_func)
    paths = map(i -> Vector{Node{State}}(), 1:N)
    costs_table = Vector{Float64}([0.0])
    for i in (order_randomize ? randperm(N) : (1:N))
        if timeover()
            return (nothing, roadmaps)
        end

        if VERBOSE > 1
            @info @sprintf("\t\tstart planning for agent-%d", i)
        end

        # setup functions
        invalid =
            (S_from::SearchNode{State}, S_to::SearchNode{State}) -> begin
                t = S_to.t

                # avoid infinite search
                if t > max_makespan
                    return true
                end

                # check collisions
                for j = 1:N
                    if j == i || isempty(paths[j])
                        continue
                    end
                    l = length(paths[j])
                    q_j_from = paths[j][min(t - 1, l)].q
                    q_j_to = paths[j][min(t, l)].q
                    if collide(S_from.v.q, S_to.v.q, q_j_from, q_j_to, i, j)
                        return true
                    end
                end

                return false
            end

        check_goal_i =
            (S::SearchNode{State}) -> begin
                if !check_goal(S.v, i)
                    return false
                end

                # check additional constraints
                for j = 1:N
                    if j == i || isempty(paths[j])
                        continue
                    end

                    for t = S.t+1:length(paths[j])
                        if collide(S.v.q, S.v.q, paths[j][t-1].q, paths[j][t].q, i, j)
                            return false
                        end
                    end
                end

                return true
            end
        h_func_i = (v::Node{State}) -> distance_tables[i][v.id]

        # accurate cost calculation
        g_func_i =
            (S::SearchNode{State}, q::State) -> begin
                cost_to_come_S =
                    max(S.g, isassigned(costs_table, S.t) ? costs_table[S.t] : 0)
                return max(
                    cost_to_come_S + g_func(S.v.q, q, i),
                    isassigned(costs_table, S.t + 1) ? costs_table[S.t+1] : 0,
                )
            end

        path = find_timed_path(
            roadmaps[i],
            invalid,
            check_goal_i,
            h_func_i,
            g_func_i;
            TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
        )

        # case failure
        if isnothing(path)
            if VERBOSE > 1
                @info @sprintf("\t\tfailed planning for agent-%d", i)
            end
            return (nothing, roadmaps)
        end

        paths[i] = path

        # update costs table
        for t = 2:length(path)
            c_new = max(
                isassigned(costs_table, t) ? costs_table[t] : 0,
                costs_table[t-1] + g_func(path[t-1].q, path[t].q),
            )
            if isassigned(costs_table, t)
                costs_table[t] = c_new
            else
                push!(costs_table, c_new)
            end
        end
    end

    return (convert_paths_to_configurations(paths), roadmaps)
end

function PP(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function;
    g_func::Function = gen_g_func(stay_penalty = 0.1),
    num_vertices::Int64 = 100,
    rad::Union{Nothing,Real} = nothing,
    rads::Union{Vector{Nothing},Vector{Float64}} = fill(rad, length(config_init)),
    roadmaps_growing_rate::Union{Nothing,Float64} = nothing,
    max_makespan::Int64 = 20,
    order_randomize::Bool = true,
    TIME_LIMIT::Union{Nothing,Real} = nothing,
    VERBOSE::Int64 = 0,
)::Tuple{
    Union{Nothing,Vector{Vector{Node{State}}}},  # solution
    Vector{Vector{Node{State}}},  # roadmaps
} where {State<:AbsState}

    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    roadmaps = PRMs(
        config_init,
        config_goal,
        connect,
        num_vertices;
        rads = rads,
        TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
    )
    if VERBOSE > 0
        @info @sprintf("\tconstruct initial roadmaps: |V|=%d", num_vertices)
    end

    if timeover()
        if VERBOSE > 0
            @info @sprintf("\tsolution is not found within time limit")
        end
        return (nothing, roadmaps)
    end

    solution, roadmaps = PP(
        roadmaps,
        collide,
        check_goal,
        g_func;
        max_makespan = max_makespan,
        order_randomize = order_randomize,
        VERBOSE = VERBOSE,
        TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
    )

    while isnothing(solution) && !isnothing(roadmaps_growing_rate) && !timeover()
        num_vertices = Int64(floor(num_vertices * roadmaps_growing_rate))
        if VERBOSE > 0
            @info @sprintf("\tupdate roadmaps: |V|=%d", num_vertices)
        end
        roadmaps = PRMs!(
            roadmaps,
            connect,
            num_vertices;
            rads = rads,
            TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
        )
        if timeover()
            break
        end
        solution, roadmaps = PP(
            roadmaps,
            collide,
            check_goal,
            g_func;
            max_makespan = max_makespan,
            order_randomize = order_randomize,
            VERBOSE = VERBOSE,
            TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
        )
    end

    if VERBOSE > 0
        if isnothing(solution)
            @info "\tfailed to find solution"
        else
            @info "\tfound solution"
        end
    end
    return (solution, roadmaps)
end

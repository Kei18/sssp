function prioritized_planning(
    roadmaps::Vector{Vector{Node{State}}},
    collide::Function,
    check_goal::Function,
    g_func::Function;
    max_makespan::Int64=20,
    VERBOSE::Int64=0,
    )::Tuple{
        Union{Nothing, Vector{Vector{Node{State}}}},  # solution
        Vector{Vector{Node{State}}},  # roadmaps
    } where {State<:AbsState}

    distance_tables = get_distance_tables(roadmaps, g_func)

    paths = Vector{Vector{Node{State}}}()
    N = length(roadmaps)
    for i = 1:N
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
                for j = 1:i-1
                    l = length(paths[j])
                    q_j_from = paths[j][min(t - 1, l)].q
                    q_j_to = paths[j][min(t, l)].q
                    if collide(S_from.v.q, S_to.v.q, q_j_from, q_j_to, i, j)
                        return true
                    end
                end

                return false
            end

        check_goal_i = (S::SearchNode{State}) -> check_goal(S.v, i)
        h_func_i = (v::Node{State}) -> distance_tables[i][v.id]
        g_func_i = (q_from::State, q_to::State) -> g_func(q_from, q_to, i)

        path = find_timed_path(roadmaps[i], invalid, check_goal_i, h_func_i, g_func_i)

        # case failure
        if path == nothing
            return (nothing, roadmaps)
        end

        push!(paths, path)
    end

    return (convert_paths_to_configurations(paths), roadmaps)
end

function prioritized_planning(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function,
    g_func::Function;
    num_vertices::Int64 = 100,
    rad::Union{Nothing, Real} = nothing,
    rads::Union{Vector{Nothing}, Vector{Float64}} = fill(rad, length(config_init)),
    roadmaps_growing_rate::Union{Nothing, Float64}=nothing,
    max_makespan::Int64=20,
    VERBOSE::Int64=0,
    )::Tuple{
        Union{Nothing, Vector{Vector{Node{State}}}},  # solution
        Vector{Vector{Node{State}}},  # roadmaps
    } where {State<:AbsState}

    roadmaps = PRMs(config_init, config_goal, connect, num_vertices; rads=rads)
    if VERBOSE > 0
        @info @sprintf("\tconstruct initial roadmaps: |V|=%d", num_vertices)
    end

    solution, roadmaps = prioritized_planning(
        roadmaps, collide, check_goal, g_func; max_makespan=max_makespan, VERBOSE=VERBOSE)

    while solution == nothing && roadmaps_growing_rate != nothing
        num_vertices = Int64(floor(num_vertices * roadmaps_growing_rate))
        if VERBOSE > 0
            @info @sprintf("\tupdate roadmaps: |V|=%d", num_vertices)
        end
        roadmaps = PRMs!(roadmaps, connect, num_vertices; rads=rads)
        solution, roadmaps = prioritized_planning(
            roadmaps, collide, check_goal, g_func; max_makespan=max_makespan, VERBOSE=VERBOSE)
    end

    if VERBOSE > 0
        @info "\tfound solution"
    end
    return (solution, roadmaps)
end

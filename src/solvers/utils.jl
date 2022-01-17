@kwdef struct SearchNode{State<:AbsState}
    v::Node{State}
    t::Int64
    id::String = @sprintf("%d-%d", v.id, t)
    parent_id::Union{Nothing,String} = nothing
    g::Float64 = 0.0
    h::Float64 = 0.0
    f::Float64 = g + h
    unique_id::Int64 = 1
end

function find_timed_path(
    roadmap::Vector{Node{State}},
    invalid::Function,
    check_goal::Function,
    h_func::Function,
    g_func::Function;
    TIME_LIMIT::Union{Nothing,Float64} = nothing,
)::Union{Nothing,Vector{Node{State}}} where {State<:AbsState}

    # timeout
    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    CLOSE = Dict{String,SearchNode{State}}()
    OPEN = PriorityQueue{SearchNode{State},Float64}()

    num_generated_nodes = 1

    # initial node
    v_init = roadmap[1]
    S_init =
        SearchNode(v = v_init, t = 1, h = h_func(v_init), unique_id = num_generated_nodes)
    # case: not connected
    if S_init.h == typemax(Float64)
        return nothing
    end
    enqueue!(OPEN, S_init, S_init.f)

    # main loop
    while !isempty(OPEN) && !timeover()

        # pop
        S = dequeue!(OPEN)
        if haskey(CLOSE, S.id)
            continue
        end
        CLOSE[S.id] = S

        # check goal condition
        if check_goal(S)
            path = []
            while S.parent_id != nothing
                pushfirst!(path, S.v)
                S = CLOSE[S.parent_id]
            end
            pushfirst!(path, v_init)
            return path
        end

        # expand
        for k in S.v.neighbors
            u = roadmap[k]
            num_generated_nodes += 1
            S_new = SearchNode(
                v = u,
                t = S.t + 1,
                parent_id = S.id,
                g = g_func(S, u.q),
                h = h_func(u),
                unique_id = num_generated_nodes,
            )
            if haskey(CLOSE, S_new.id) || invalid(S, S_new)
                continue
            end
            enqueue!(OPEN, S_new, S_new.f)
        end
    end

    return nothing
end

function convert_paths_to_configurations(
    paths::Vector{Vector{Node{State}}},
)::Vector{Vector{Node{State}}} where {State<:AbsState}

    max_len = maximum(map(e -> length(e), paths))
    return [map(path -> path[min(t, length(path))], paths) for t = 1:max_len]
end

function get_distance_table(
    roadmap::Vector{Node{State}},
    g_func::Function,
    goal_node::Node{State} = roadmap[2],
)::Vector{Float64} where {State<:AbsState}
    """by Dijkstra"""

    table = fill(typemax(Float64), length(roadmap))
    OPEN = PriorityQueue{Int64,Float64}()

    # setup initial vertex
    table[goal_node.id] = 0
    enqueue!(OPEN, goal_node.id, 0)

    while !isempty(OPEN)
        # pop
        v_id = dequeue!(OPEN)
        v = roadmap[v_id]
        d = table[v_id]

        # expand
        for u_id in v.neighbors
            u = roadmap[u_id]
            g = g_func(v.q, u.q) + d
            # update
            if g < table[u_id]
                if haskey(OPEN, u_id)
                    delete!(OPEN, u_id)
                end
                table[u_id] = g
                enqueue!(OPEN, u_id, g)
            end
        end
    end

    return table
end

function get_distance_tables(
    roadmaps::Vector{Vector{Node{State}}},
    g_func::Function,
)::Vector{Vector{Float64}} where {State<:AbsState}

    return map(rmp -> get_distance_table(rmp, g_func), roadmaps)
end

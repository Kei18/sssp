"""utilities of solvers"""

"""
    SearchNode{State<:AbsState}

used in space-timed A*
"""
@kwdef struct SearchNode{State<:AbsState}
    v::Node{State}  # corresponding vertex in roadmap
    t::Int64  # time
    id::String = @sprintf("%d-%d", v.id, t)  # id of search node
    parent_id::Union{Nothing,String} = nothing  # parent in the search tree
    g::Float64 = 0.0  # g-value
    h::Float64 = 0.0  # h-value
    f::Float64 = g + h  # f-value
    unique_id::Int64 = 1   # to avoid duplication
end

"""
    gen_g_func(;
        greedy::Bool = false,           # true -> returns constant function that always returns 0
        stay_penalty::Float64 = 0.0     # adds cost for "stay" actions
    )::Function

generate cost function based on distance (g-value)
"""
function gen_g_func(; greedy::Bool = false, stay_penalty::Float64 = 0.0)::Function
    f(Q_from::Vector, Q_to::Vector)::Float64 = begin
        greedy ? 0 : dist(Q_from, Q_to)
    end

    f(q_from, q_to, i::Int64 = 1)::Float64 = begin
        greedy ? 0 : (dist(q_from, q_to) + (q_from == q_to ? stay_penalty : 0))
    end

    return f
end

"""
    find_timed_path(
        roadmap::Vector{Node{State}},   # roadmap
        invalid::Function,              # collision check function
        check_goal::Function,           # goal check function
        h_func::Function,               # heuristic function
        g_func::Function;               # cost function
        TIME_LIMIT::Union{Nothing,Float64} = nothing,  # time limit (sec)
    )::Union{
        Nothing,                        # case not found
        Vector{Node{State}}             # path
    } where {State<:AbsState}

perform space-time A*
"""
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

    # setup initial node
    v_init = roadmap[1]
    S_init =
        SearchNode(v = v_init, t = 1, h = h_func(v_init), unique_id = num_generated_nodes)
    # case: not connected
    S_init.h == typemax(Float64) && return nothing
    # insert initial node
    enqueue!(OPEN, S_init, S_init.f)

    # main loop
    while !isempty(OPEN) && !timeover()

        # pop
        S = dequeue!(OPEN)
        haskey(CLOSE, S.id) && continue
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
            (haskey(CLOSE, S_new.id) || invalid(S, S_new)) && continue
            enqueue!(OPEN, S_new, S_new.f)
        end
    end

    return nothing
end

"""
    convert_paths_to_configurations(
        paths::Vector{Vector{Node{State}}},
    )::Vector{Vector{Node{State}}} where {State<:AbsState}

transform paths (vector of paths) to configurations (vector of configurations)
"""
function convert_paths_to_configurations(
    paths::Vector{Vector{Node{State}}},
)::Vector{Vector{Node{State}}} where {State<:AbsState}

    max_len = maximum(map(e -> length(e), paths))
    return [map(path -> path[min(t, length(path))], paths) for t = 1:max_len]
end

"""

    get_distance_table(
        roadmap::Vector{Node{State}};
        g_func::Function = dist,
        goal_node::Node{State} = roadmap[2],
    )::Vector{Float64} where {State<:AbsState}

given one roadamp, compute distance table by Dijkstra method
"""
function get_distance_table(
    roadmap::Vector{Node{State}};
    g_func::Function = dist,
    goal_node::Node{State} = roadmap[2],
)::Vector{Float64} where {State<:AbsState}

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
            # update distance
            if g < table[u_id]
                haskey(OPEN, u_id) && delete!(OPEN, u_id)
                table[u_id] = g
                enqueue!(OPEN, u_id, g)
            end
        end
    end

    return table
end

"""dubins case requires neighbor computation"""
function get_distance_table(
    roadmap::Vector{Node{MRMP.StateDubins}};
    g_func::Function = dist,
    goal_node::Node{MRMP.StateDubins} = roadmap[2],
)::Vector{Float64}

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
        for u in filter(w -> v_id in w.neighbors, roadmap)
            g = g_func(v.q, u.q) + d
            # update distance
            if g < table[u.id]
                haskey(OPEN, u.id) && delete!(OPEN, u.id)
                table[u.id] = g
                enqueue!(OPEN, u.id, g)
            end
        end
    end

    return table
end


"""
    get_distance_tables(
        roadmaps::Vector{Vector{Node{State}}};
        g_func::Function = dist,
        goal_nodes::Vector{Node{State}} = map(rmp -> rmp[2], roadmaps),
    )::Vector{Vector{Float64}} where {State<:AbsState}

compute distance tables for multiple roadmaps by Dijkstra method
"""
function get_distance_tables(
    roadmaps::Vector{Vector{Node{State}}};
    g_func::Function = dist,
    goal_nodes::Vector{Node{State}} = map(rmp -> rmp[2], roadmaps),
)::Vector{Vector{Float64}} where {State<:AbsState}

    return map(
        i -> get_distance_table(roadmaps[i]; g_func = g_func, goal_node = goal_nodes[i]),
        1:length(roadmaps),
    )
end

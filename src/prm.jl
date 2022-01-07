function construct_PRM(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    uniform_sampling::Function,
    num_vertices::Int64=100,
    rad::Float64=0.2
    )::Vector{Vector{Node{State}}} where {State<:AbsState}

    N = length(config_init)
    roadmaps = map(
        e -> [Node{State}(e[1], 1, []), Node{State}(e[2], 2, [])],
        zip(config_init, config_goal),
    )

    # sampling vertices
    for i = 1:N
        while length(roadmaps[i]) < num_vertices
            q = uniform_sampling()
            if connect(q, q, i)
                push!(roadmaps[i], Node{State}(q, length(roadmaps[i])+1, []))
            end
        end
    end

    # identify neighbors
    for i = 1:N
        for j = 1:num_vertices, k = 1:num_vertices
            q_from = roadmaps[i][j].q
            q_to = roadmaps[i][k].q
            if connect(q_from, q_to, i; ignore_eps=true) && dist(q_from, q_to) <= rad
                push!(roadmaps[i][j].neighbors, k)
            end
        end
    end

    return roadmaps
end

function get_distance_table(
    roadmap::Vector{Node{State}},
    g_func::Function,
    goal_node::Node{State}=roadmap[2],
    )::Vector{Float64} where State<:AbsState
    """by Dijkstra"""

    table = fill(10000.0, length(roadmap))
    OPEN = PriorityQueue{Int64, Float64}()

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
    )::Vector{Vector{Float64}} where State<:AbsState

    return map(rmp -> get_distance_table(rmp, g_func), roadmaps)
end


@kwdef struct SearchNode{State<:AbsState}
    v::Node{State}
    t::Int64
    id::String = @sprintf("%d-%d", v.id, t)
    parent_id::Union{Nothing, String} = nothing
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
    g_func::Function,
    )::Vector{Node{State}} where {State<:AbsState}

    CLOSE = Dict{String, SearchNode{State}}()
    OPEN = PriorityQueue{SearchNode{State}, Float64}()

    num_generated_nodes = 1

    # initial node
    v_init = roadmap[1]
    S_init = SearchNode(v=v_init, t=1, h=h_func(v_init), unique_id=num_generated_nodes)
    enqueue!(OPEN, S_init, S_init.f)

    # main loop
    while !isempty(OPEN)

        # pop
        S = dequeue!(OPEN)
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
                v=u,
                t=S.t+1,
                parent_id=S.id,
                g=S.g+g_func(S.v.q, u.q),
                h=h_func(u),
                unique_id=num_generated_nodes,
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
    paths::Vector{Vector{Node{State}}}
    )::Vector{Vector{Node{State}}} where State<:AbsState

    max_len = maximum(map(e -> length(e), paths))
    return [map(path -> path[min(t, length(path))], paths) for t = 1:max_len]
end

function prioritized_planning(
    roadmaps::Vector{Vector{Node{State}}},
    collide::Function,
    check_goal::Function,
    g_func::Function
    )::Vector{Vector{Node{State}}} where {State<:AbsState}

    distance_tables = get_distance_tables(roadmaps, g_func)

    paths = Vector{Vector{Node{State}}}()
    N = length(roadmaps)
    for i = 1:N
        # setup functions
        invalid = (S_from::SearchNode{State}, S_to::SearchNode{State}) -> begin
            t = S_to.t

            # check collisions
            for j = 1:i-1
                l = length(paths[j])
                q_j_from = paths[j][min(t-1, l)].q
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
            return nothing
        end

        push!(paths, path)
    end

    return convert_paths_to_configurations(paths)
end

abstract type Constraint end

struct VertexConstraint{State<:AbsState} <:Constraint
    agent::Int64
    v::Node{State}
    t::Int64
end

struct EdgeConstraint{State<:AbsState} <:Constraint
    agent::Int64
    v_from::Node{State}
    v_to::Node{State}
    t_to::Int64
end

@kwdef struct HighLevelNode{State<:AbsState}
    paths::Vector{Vector{Node{State}}} = Vector{Vector{Node{State}}}()
    constraints::Vector{Constraint} = Vector{Constraint}()
    f::Float64 = 0.0
    valid::Bool = true
end

function conflict_based_search(
    roadmaps::Vector{Vector{Node{State}}},
    collide::Function,
    check_goal::Function,
    g_func::Function,
    f_func_highlevel::Function = (paths) -> Float64(get_num_collisions(paths, collide));
    VERBOSE::Int64=0,
    )::Union{Nothing, Vector{Vector{Node{State}}}} where {State<:AbsState}

    # compute distance tables
    distance_tables = get_distance_tables(roadmaps, g_func)

    # for high-level search
    OPEN = PriorityQueue{HighLevelNode{State}, Float64}()

    # setup initial node
    init_node = get_init_node(roadmaps, check_goal, distance_tables, g_func, f_func_highlevel)
    enqueue!(OPEN, init_node, init_node.f)

    # main loop
    while !isempty(OPEN)
        # pop
        node = dequeue!(OPEN)

        # check constraints
        constraints = get_constraints(node.paths, collide)
        if isempty(constraints)
            return convert_paths_to_configurations(node.paths)
        end

        # create new nodes
        for c in constraints
            new_node = invoke(
                node, c, roadmaps, collide, check_goal, distance_tables, g_func, f_func_highlevel
            )
            if new_node.valid
                enqueue!(OPEN, new_node, new_node.f)
                if VERBOSE > 0
                    if typeof(c) == VertexConstraint{State}
                        @info @sprintf(
                            "\tadd new child, agent-%d at t=%d, state=%s",
                            c.agent, c.t, to_string(c.v.q)
                        )
                    elseif typeof(c) == EdgeConstraint{State}
                        @info @sprintf(
                            "\tadd new child, agent-%d at t: %d -> %d, state: %s -> %s",
                            c.agent, c.t_to-1, c.t_to, to_string(c.v_from.q), to_string(c.v_to.q)
                        )
                    end
                end
            end
        end
    end

    return nothing
end

function invoke(
    node::HighLevelNode{State},
    new_constraint::Constraint,
    roadmaps::Vector{Vector{Node{State}}},
    collide::Function,
    check_goal::Function,
    distance_tables::Vector{Vector{Float64}},
    g_func::Function,
    f_func_highlevel::Function,
    )::HighLevelNode{State} where {State<:AbsState}

    N = length(roadmaps)
    i = new_constraint.agent

    constraints_i = filter(c -> c.agent == i, node.constraints)
    push!(constraints_i, new_constraint)

    # setup search function
    invalid = (S_from::SearchNode{State}, S_to::SearchNode{State}) -> begin
        for c in constraints_i
            if typeof(c) == VertexConstraint{State}
                if c.t == S_to.t && c.v == S_to.v
                    return true
                end
            elseif typeof(c) == EdgeConstraint{State}
                if c.t_to == S_to.t && c.v_to == S_to.v && c.v_from == S_from.v
                    return true
                end
            end
        end
        return false
    end

    check_goal_i = (S::SearchNode{State}) -> begin
        if !check_goal(S.v, i)
            return false
        end

        # check additional constraints
        if any(c -> typeof(c) == VertexConstraint{State} && c.t >= S.t && c.v == S.v, constraints_i)
            return false
        end

        return true
    end

    h_func_i = (v::Node{State}) -> distance_tables[i][v.id]
    g_func_i = (q_from::State, q_to::State) -> g_func(q_from, q_to, i)

    new_path = find_timed_path(roadmaps[i], invalid, check_goal_i, h_func_i, g_func_i)

    # failed
    if new_path == nothing
        return HighLevelNode(valid=false)
    end

    constraints = vcat(deepcopy(node.constraints), [new_constraint])
    paths = map(e -> e[1] != i ? deepcopy(e[2]) : new_path, enumerate(node.paths))
    return HighLevelNode(paths=paths, constraints=constraints, f=f_func_highlevel(paths))
end

function get_constraints(
    paths::Vector{Vector{Node{State}}},
    collide::Function;
    check_all_collisions::Bool=false,
    )::Vector{Constraint} where State<:AbsState

    constraints = Vector{Constraint}()

    N = length(paths)
    max_len = maximum(map(e -> length(e), paths))
    for t = 2:max_len
        for i = 1:N, j = i+1:N
            l_i = length(paths[i])
            l_j = length(paths[j])
            v_i_to = paths[i][min(t, l_i)]
            v_j_to = paths[j][min(t, l_j)]

            # check vertex conflict
            if collide(v_i_to.q, v_j_to.q, i, j)
                push!(constraints, VertexConstraint{State}(i, v_i_to, t))
                push!(constraints, VertexConstraint{State}(j, v_j_to, t))
                if !check_all_collisions
                    return constraints
                end
                continue
            end

            # check edge conflict
            v_i_from = paths[i][min(t-1, l_i)]
            v_j_from = paths[j][min(t-1, l_j)]
            if collide(v_i_from.q, v_i_to.q, v_j_from.q, v_j_to.q, i, j)
                push!(constraints, EdgeConstraint{State}(i, v_i_from, v_i_to, t))
                push!(constraints, EdgeConstraint{State}(j, v_j_from, v_j_to, t))
                if !check_all_collisions
                    return constraints
                end
                continue
            end
        end
    end

    return constraints
end

function get_init_node(
    roadmaps::Vector{Vector{Node{State}}},
    check_goal::Function,
    distance_tables::Vector{Vector{Float64}},
    g_func::Function,
    f_func_highlevel::Function,
    )::HighLevelNode{State} where State<:AbsState

    invalid = (S_from::SearchNode{State}, S_to::SearchNode{State}) -> false

    paths = Vector{Vector{Node{State}}}()
    for i in 1:length(roadmaps)
        check_goal_i = (S::SearchNode{State}) -> check_goal(S.v, i)
        h_func_i = (v::Node{State}) -> distance_tables[i][v.id]
        g_func_i = (q_from::State, q_to::State) -> g_func(q_from, q_to, i)
        path = find_timed_path(roadmaps[i], invalid, check_goal_i, h_func_i, g_func_i)

        # failure
        if path == nothing
            return HighLevelNode{State}(valid=false)
        end

        push!(paths, path)
    end

    return HighLevelNode{State}(paths=paths, f=f_func_highlevel(paths))
end

function get_num_collisions(
    paths::Vector{Vector{Node{State}}},
    collide::Function
    )::Int64 where State<:AbsState

    constraints = get_constraints(paths, collide; check_all_collisions=true)
    return length(constraints) / 2
end

module CBS
export conflict_based_search

import ...MRMP: AbsState, Node, to_string
import ..Solvers:
    SearchNode, find_timed_path, convert_paths_to_configurations, get_distance_tables
import Printf: @sprintf
import Base: @kwdef
import DataStructures: PriorityQueue, enqueue!, dequeue!

abstract type Constraint end

struct VertexConstraint{State<:AbsState} <: Constraint
    agent::Int64
    v::Node{State}
    t::Int64
end

struct EdgeConstraint{State<:AbsState} <: Constraint
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
    VERBOSE::Int64 = 0,
)::Union{Nothing,Vector{Vector{Node{State}}}} where {State<:AbsState}

    # compute distance tables
    distance_tables = get_distance_tables(roadmaps, g_func)

    # for high-level search
    OPEN = PriorityQueue{HighLevelNode{State},Float64}()

    # setup initial node
    init_node =
        get_init_node(roadmaps, check_goal, distance_tables, g_func, f_func_highlevel)
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
                node,
                c,
                roadmaps,
                collide,
                check_goal,
                distance_tables,
                g_func,
                f_func_highlevel,
            )
            if new_node.valid
                enqueue!(OPEN, new_node, new_node.f)
                if VERBOSE > 0
                    if typeof(c) == VertexConstraint{State}
                        @info @sprintf(
                            "\tadd new child, agent-%d at t=%d, state=%s",
                            c.agent,
                            c.t,
                            to_string(c.v.q)
                        )
                    elseif typeof(c) == EdgeConstraint{State}
                        @info @sprintf(
                            "\tadd new child, agent-%d at t: %d -> %d, state: %s -> %s",
                            c.agent,
                            c.t_to - 1,
                            c.t_to,
                            to_string(c.v_from.q),
                            to_string(c.v_to.q)
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
    invalid =
        (S_from::SearchNode{State}, S_to::SearchNode{State}) -> begin
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

    check_goal_i =
        (S::SearchNode{State}) -> begin
            if !check_goal(S.v, i)
                return false
            end

            # check additional constraints
            if any(
                c -> typeof(c) == VertexConstraint{State} && c.t >= S.t && c.v == S.v,
                constraints_i,
            )
                return false
            end

            return true
        end

    h_func_i = (v::Node{State}) -> distance_tables[i][v.id]
    g_func_i = (q_from::State, q_to::State) -> g_func(q_from, q_to, i)

    new_path = find_timed_path(roadmaps[i], invalid, check_goal_i, h_func_i, g_func_i)

    # failed
    if new_path == nothing
        return HighLevelNode(valid = false)
    end

    constraints = vcat(deepcopy(node.constraints), [new_constraint])
    paths = map(e -> e[1] != i ? deepcopy(e[2]) : new_path, enumerate(node.paths))
    return HighLevelNode(
        paths = paths,
        constraints = constraints,
        f = f_func_highlevel(paths),
    )
end

function get_constraints(
    paths::Vector{Vector{Node{State}}},
    collide::Function;
    check_all_collisions::Bool = false,
)::Vector{Constraint} where {State<:AbsState}

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
            v_i_from = paths[i][min(t - 1, l_i)]
            v_j_from = paths[j][min(t - 1, l_j)]
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
)::HighLevelNode{State} where {State<:AbsState}

    invalid = (S_from::SearchNode{State}, S_to::SearchNode{State}) -> false

    paths = Vector{Vector{Node{State}}}()
    for i = 1:length(roadmaps)
        check_goal_i = (S::SearchNode{State}) -> check_goal(S.v, i)
        h_func_i = (v::Node{State}) -> distance_tables[i][v.id]
        g_func_i = (q_from::State, q_to::State) -> g_func(q_from, q_to, i)
        path = find_timed_path(roadmaps[i], invalid, check_goal_i, h_func_i, g_func_i)

        # failure
        if path == nothing
            return HighLevelNode{State}(valid = false)
        end

        push!(paths, path)
    end

    return HighLevelNode{State}(paths = paths, f = f_func_highlevel(paths))
end

function get_num_collisions(
    paths::Vector{Vector{Node{State}}},
    collide::Function,
)::Int64 where {State<:AbsState}

    constraints = get_constraints(paths, collide; check_all_collisions = true)
    return length(constraints) / 2
end

end

module LibCBS
export CBS

import ...MRMP: AbsState, Node, now, elapsed_sec
import ..Solvers:
    SearchNode,
    find_timed_path,
    convert_paths_to_configurations,
    get_distance_tables,
    gen_g_func,
    PRMs,
    PRMs!
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

function CBS(
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
    collision_weight::Float64 = 1.0,
    VERBOSE::Int64 = 0,
    TIME_LIMIT::Union{Nothing,Real} = nothing,
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
        TIME_LIMIT = isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed(),
    )
    VERBOSE > 0 && @info @sprintf("\tconstruct initial roadmaps: |V|=%d", num_vertices)

    if timeover()
        VERBOSE > 0 && @info @sprintf("\tsolution is not found within time limit")
        return (nothing, roadmaps)
    end

    solution, roadmaps = conflict_based_search(
        roadmaps,
        collide,
        check_goal,
        g_func;
        max_makespan = max_makespan,
        collision_weight = collision_weight,
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
        timeover() && break
        solution, roadmaps = conflict_based_search(
            roadmaps,
            collide,
            check_goal,
            g_func;
            max_makespan = max_makespan,
            collision_weight = collision_weight,
            VERBOSE = VERBOSE,
            TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
        )
    end

    VERBOSE > 0 &&
        @info (!isnothing(solution) ? "\tfound solution" : "\tfail to find solution")
    return (solution, roadmaps)
end

function conflict_based_search(
    roadmaps::Vector{Vector{Node{State}}},
    collide::Function,
    check_goal::Function,
    g_func::Function,
    f_func_highlevel::Function = (paths) -> Float64(get_num_collisions(paths, collide));
    max_makespan::Int64 = 20,
    collision_weight::Float64 = 1.0,
    VERBOSE::Int64 = 0,
    TIME_LIMIT::Union{Nothing,Real} = nothing,
)::Tuple{
    Union{Nothing,Vector{Vector{Node{State}}}},  # solution
    Vector{Vector{Node{State}}},  # roadmaps
} where {State<:AbsState}

    # timeout
    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    print_constraint!(c::Constraint) = begin
        if VERBOSE > 2
            if typeof(c) == VertexConstraint{State}
                @info @sprintf(
                    "\t\t\tadd new child, agent-%d at t=%d, state=%s",
                    c.agent,
                    c.t,
                    c.v.q
                )
            elseif typeof(c) == EdgeConstraint{State}
                @info @sprintf(
                    "\t\t\tadd new child, agent-%d at t: %d -> %d, state: %s -> %s",
                    c.agent,
                    c.t_to - 1,
                    c.t_to,
                    c.v_from.q,
                    c.v_to.q
                )
            end
        end
    end

    # compute distance tables
    distance_tables = get_distance_tables(roadmaps, g_func)

    # for high-level search
    OPEN = PriorityQueue{HighLevelNode{State},Float64}()

    # setup initial node
    init_node = get_init_node(
        roadmaps,
        check_goal,
        distance_tables,
        g_func,
        f_func_highlevel;
        TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
    )
    if init_node.valid
        enqueue!(OPEN, init_node, init_node.f)
    end

    # main loop
    iter = 0
    while !isempty(OPEN) && !timeover()

        iter += 1

        # pop
        node = dequeue!(OPEN)

        VERBOSE > 1 && @info @sprintf("\t\titer=%04d, expand new node, f=%d", iter, node.f)

        # check constraints
        constraints = get_constraints(node.paths, collide)
        isempty(constraints) &&
            return (convert_paths_to_configurations(node.paths), roadmaps)

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
                f_func_highlevel;
                max_makespan = max_makespan,
                collision_weight = collision_weight,
                TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
            )
            if new_node.valid
                enqueue!(OPEN, new_node, new_node.f)
                print_constraint!(c)
            end
        end
    end

    return (nothing, roadmaps)
end

function invoke(
    node::HighLevelNode{State},
    new_constraint::Constraint,
    roadmaps::Vector{Vector{Node{State}}},
    collide::Function,
    check_goal::Function,
    distance_tables::Vector{Vector{Float64}},
    g_func::Function,
    f_func_highlevel::Function;
    max_makespan::Int64 = 20,
    collision_weight::Float64 = 0,
    TIME_LIMIT::Union{Nothing,Float64} = nothing,
)::HighLevelNode{State} where {State<:AbsState}

    N = length(roadmaps)
    i = new_constraint.agent

    constraints_i = filter(c -> c.agent == i, node.constraints)
    push!(constraints_i, new_constraint)

    # setup search function
    invalid =
        (S_from::SearchNode{State}, S_to::SearchNode{State}) -> begin
            # avoid infinite search
            S_to.t > max_makespan && return true

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
            !check_goal(S.v, i) && return false

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

    # create costs table
    max_len = maximum(map(e -> length(e), node.paths))
    costs_table = fill(0.0, max_len)
    for t = 2:max_len
        for (j, path) in enumerate(node.paths)
            if j == i || !isassigned(path, t)
                continue
            end

            costs_table[t] =
                max(costs_table[t-1] + g_func(path[t-1].q, path[t].q, i), costs_table[t])
        end
    end

    g_func_i =
        (S::SearchNode{State}, q_i_to::State) -> begin
            t = S.t + 1
            q_i_from = S.v.q
            cost_to_come_S =
                max(S.g, isassigned(costs_table, t - 1) ? costs_table[t-1] : 0)
            cost_to_come_q = max(
                cost_to_come_S + g_func(q_i_from, q_i_to, i),
                isassigned(costs_table, t) ? costs_table[t] : 0,
            )
            num_collsions = count(
                j -> begin
                    path = node.paths[j]
                    q_j_to = path[min(t, length(path))].q
                    q_j_from = path[min(t - 1, length(path))].q
                    return (
                        collide(q_j_to, q_i_to, j, i) ||
                        collide(q_j_from, q_j_to, q_i_from, q_i_to, j, i)
                    )
                end,
                filter(j -> j != i, 1:N),
            )
            return cost_to_come_q + num_collsions * collision_weight
        end
    new_path = find_timed_path(
        roadmaps[i],
        invalid,
        check_goal_i,
        h_func_i,
        g_func_i;
        TIME_LIMIT = TIME_LIMIT,
    )

    # failed
    isnothing(new_path) && return HighLevelNode{State}(valid = false)

    constraints = vcat(node.constraints, [new_constraint])
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
                !check_all_collisions & return constraints
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
    f_func_highlevel::Function;
    TIME_LIMIT::Union{Nothing,Float64} = nothing,
)::HighLevelNode{State} where {State<:AbsState}

    t_s = now()
    elapsed() = elapsed_sec(t_s)

    invalid = (S_from::SearchNode{State}, S_to::SearchNode{State}) -> false

    paths = Vector{Vector{Node{State}}}()
    for i = 1:length(roadmaps)
        check_goal_i = (S::SearchNode{State}) -> check_goal(S.v, i)
        h_func_i = (v::Node{State}) -> distance_tables[i][v.id]
        g_func_i = (S::SearchNode{State}, q::State) -> S.g + g_func(S.v.q, q, i)
        path = find_timed_path(
            roadmaps[i],
            invalid,
            check_goal_i,
            h_func_i,
            g_func_i;
            TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
        )
        # failure
        isnothing(path) && return HighLevelNode{State}(valid = false)

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

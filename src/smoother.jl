"""
smoother of solution

ref:
- Dechter, R., Meiri, I., and Pearl, J. (1991).
  Temporal constraint networks.
  Artificial Intelligence (AIJ).
- Honing, W. et al. (2016).
  Multi-agent path finding with kinematic constraints.
  In ICAPS.
"""

mutable struct Action{State<:AbsState}
    id::String
    from::Node{State}
    to::Node{State}
    t::Int64
    agent::Int64
    predecessors::Vector{Tuple{Int64,String}}
    successors::Vector{Tuple{Int64,String}}
end

function get_action_id(
    from::Node{State},
    to::Node{State},
    t::Int64,
)::String where {State<:AbsState}
    return string(from.id) * "_" * string(to.id) * "_" * string(t)
end

"""create temporal plan graph temporal plan graph"""
function get_temporal_plan_graph(
    solution::Vector{Vector{Node{State}}},
    collide::Function,
    connect::Function;
    skip_connection::Bool = true,
)::Vector{Vector{Action{State}}} where {State<:AbsState}

    N = length(solution[1])
    T = length(solution)

    # temporal plan graph
    TPG = [Vector{Action{State}}() for i = 1:N]

    # type-1 dependency
    for i = 1:N
        v_last = solution[1][i]
        for (t, Q) in enumerate(solution[1:end])
            v_current = Q[i]

            if v_current.id != v_last.id
                action = Action(
                    get_action_id(v_last, v_current, t),
                    v_last,
                    v_current,
                    t,
                    i,
                    Vector{Tuple{Int64,String}}(),
                    Vector{Tuple{Int64,String}}(),
                )

                # add type-1 dependency
                if length(TPG[i]) > 0
                    action_pre = TPG[i][end]
                    push!(action.predecessors, (i, action_pre.id))
                end

                # insert
                push!(TPG[i], action)

                # update
                v_last = v_current
            end
        end
    end

    # type-2 dependency
    for i = 1:N
        for action_self in TPG[i]
            for j = 1:N
                if j == i
                    continue
                end

                # exclude ealier actions
                for action_other in filter(a -> a.t > action_self.t, TPG[j])

                    # create type-2 dependency
                    if collide(
                        action_self.from.q,
                        action_self.to.q,
                        action_other.from.q,
                        action_other.to.q,
                        i,
                        j,
                    )
                        push!(
                            action_other.predecessors,
                            (action_self.agent, action_self.id),
                        )
                        break
                    end
                end
            end
        end
    end

    # remove redundant dependencies
    for i = 1:N
        for action in TPG[i]
            latest_actions = Dict{Int64,Action}()
            for (j, id) in action.predecessors
                action_other = TPG[j][findfirst(a -> a.id == id, TPG[j])]
                if !haskey(latest_actions, j) || latest_actions[j].t < action_other.t
                    latest_actions[j] = action_other
                end
            end
            action.predecessors = map(a -> (a.agent, a.id), values(latest_actions))

            # update successors
            for (j, id) in action.predecessors
                action_other = TPG[j][findfirst(a -> a.id == id, TPG[j])]
                push!(action_other.successors, (i, action.id))
            end
        end
    end

    skip_connection && try_skip_connection!(TPG, collide, connect)
    return TPG
end

"""remove redundant actions in temporal plan graph"""
function try_skip_connection!(
    TPG::Vector{Vector{Action{State}}},
    collide::Function,
    connect::Function,
) where {State<:AbsState}

    N = length(TPG)
    get_action = (j, id) -> TPG[j][findfirst(a -> a.id == id, TPG[j])]

    for i = 1:N
        k = 1
        while k < length(TPG[i])
            k += 1
            a1 = TPG[i][k-1]
            a2 = TPG[i][k]

            # check connection
            !connect(a1.from.q, a2.to.q, i) && continue

            # check dependencies
            any([j != i for (j, _) in a2.predecessors]) && continue
            any([j != i for (j, _) in a1.successors]) && continue

            # new action candidate
            a3 = Action(
                get_action_id(a1.from, a2.to, a1.t),
                a1.from,
                a2.to,
                a1.t,
                a1.agent,
                a1.predecessors,
                a2.successors,
            )

            # get causalities
            causal_actions = get_ancestors(a1, TPG)
            union!(causal_actions, get_descendants(a2, TPG))
            causal_actions = filter(val -> val[1] != i, causal_actions)

            # check collisions
            conflicted = false
            for j = 1:N
                if j == i
                    continue
                end
                for a4 in filter(a -> !((j, a.id) in causal_actions), TPG[j])
                    if collide(a3.from.q, a3.to.q, a4.from.q, a4.to.q, i, j)
                        conflicted = true
                        break
                    end
                end
                if conflicted
                    continue
                end
                # check last location
                if collide(a3.from.q, a3.to.q, TPG[j][end].to.q, TPG[j][end].to.q, i, j)
                    conflicted = true
                    break
                end
            end
            if conflicted
                continue
            end

            # update action orders
            deleteat!(TPG[i], k-1:k)
            insert!(TPG[i], k - 1, a3)

            # update dependencies
            for (j, id) in a3.predecessors
                a4 = get_action(j, id)
                m = findfirst(tpl -> tpl[1] == i && tpl[2] == a1.id, a4.successors)
                deleteat!(a4.successors, m)
                push!(a4.successors, (i, a3.id))
            end
            for (j, id) in a3.successors
                a4 = get_action(j, id)
                m = findfirst(tpl -> tpl[1] == i && tpl[2] == a2.id, a4.predecessors)
                deleteat!(a4.predecessors, m)
                push!(a4.predecessors, (i, a3.id))
            end
            k -= 1
        end
    end
end

"""obtain actions with causal dependencies"""
function get_causal_actions(
    action::Action{State},
    TPG::Vector{Vector{Action{State}}},
    for_ancestors::Bool,
) where {State<:AbsState}

    N = length(TPG)
    tables = [Dict{String,Set{Tuple{Int64,String}}}() for i = 1:N]
    function f(i, id)
        action = TPG[i][findfirst(a -> a.id == id, TPG[i])]
        if haskey(tables[i], id)
            return tables[i][id]
        end
        causal_actions = Set{Tuple{Int64,String}}()
        for (j, id_j) in ((for_ancestors) ? action.predecessors : action.successors)
            push!(causal_actions, (j, id_j))
            foreach(val -> push!(causal_actions, val), f(j, id_j))
        end
        tables[i][id] = causal_actions
        return causal_actions
    end
    return f(action.agent, action.id)
end

function get_ancestors(
    action::Action{State},
    TPG::Vector{Vector{Action{State}}},
) where {State<:AbsState}
    return get_causal_actions(action, TPG, true)
end

function get_descendants(
    action::Action{State},
    TPG::Vector{Vector{Action{State}}},
) where {State<:AbsState}
    return get_causal_actions(action, TPG, false)
end

"""sampling one solution from temporal plan graph"""
function get_greedy_solution(
    TPG::Vector{Vector{Action{State}}},
    config_goal::Vector{State},
)::Vector{Vector{Node{State}}} where {State<:AbsState}

    N = length(TPG)
    indexes = [1 for i = 1:N]  # internal clock of agents
    solution = [[
        (length(arr) > 0 ? arr[1].from : Node(config_goal[i], 0, Vector{Int64}())) for
        (i, arr) in enumerate(TPG)
    ]]

    while !all([k > length(TPG[i]) for (i, k) in enumerate(indexes)])
        config = []
        indexes_last = copy(indexes)
        for i = 1:N
            if length(TPG[i]) > 0
                action = TPG[i][min(indexes[i], length(TPG[i]))]
                if all([
                    findfirst(a -> a.id == id, TPG[j]) < indexes_last[j] for
                    (j, id) in action.predecessors
                ])
                    # increment
                    indexes[i] = min(indexes[i] + 1, length(TPG[i]) + 1)
                    push!(config, action.to)
                else
                    # stay
                    push!(config, action.from)
                end
            else
                push!(config, Node(config_goal[i], 0, Vector{Int64}()))
            end
        end
        push!(solution, config)
    end
    return solution
end

"""
    get_solution_cost(
        solution::Union{Nothing,Vector{Vector{Node{State}}}},
    )::Union{Nothing,Dict{Symbol,Float64}} where {State<:AbsState}

compute costs (sum_of_costs & makespan) of solution
"""
function get_solution_cost(
    solution::Union{Nothing,Vector{Vector{Node{State}}}},
)::Union{Nothing,Dict{Symbol,Float64}} where {State<:AbsState}

    isnothing(solution) && return nothing
    N = length(solution[1])
    T = length(solution)

    # compute last timesteps
    last_timesteps = fill(T, N)
    for i = 1:N
        for t in reverse(collect(1:T-1))
            solution[t][i] != solution[t+1][i] && break
            last_timesteps[i] = t
        end
    end

    makespan = 0
    sum_of_cost = 0
    for t = 2:T
        c = maximum(i -> dist(solution[t-1][i], solution[t][i]), 1:N)
        sum_of_cost += c * length(filter(e -> t <= e, last_timesteps))
        makespan += c
    end

    return Dict(:sum_of_cost => sum_of_cost, :makespan => makespan)
end

"""
    get_tpg_cost(
        TPG::Union{Nothing,Vector{Vector{Action{State}}}},
    )::Union{Nothing,Dict{Symbol,Float64}} where {State<:AbsState}

compute costs (sum_of_costs & makespan) of temporal plan graph (by dynamic programming)
"""
function get_tpg_cost(
    TPG::Union{Nothing,Vector{Vector{Action{State}}}},
)::Union{Nothing,Dict{Symbol,Float64}} where {State<:AbsState}

    isnothing(TPG) && return nothing

    N = length(TPG)
    cost_tables = [Dict{String,Float64}() for i = 1:N]

    function f(i, id)
        action = TPG[i][findfirst(a -> a.id == id, TPG[i])]
        c = dist(action.from.q, action.to.q)
        if !isempty(action.predecessors)
            c += maximum([
                (haskey(cost_tables[j], id_j) ? cost_tables[j][id_j] : f(j, id_j)) for
                (j, id_j) in action.predecessors
            ])
        end
        cost_tables[i][id] = c
        return c
    end

    arr = [(length(TPG[i]) > 0 ? f(i, TPG[i][end].id) : 0) for i = 1:N]
    return Dict(:sum_of_cost => sum(arr), :makespan => maximum(arr))
end

"""
    smoothing(
        solution::Vector{Vector{Node{State}}},
        connect::Function,
        collide::Function;
        VERBOSE::Int64 = 0,
    )::Tuple{
        Vector{Vector{Action{State}}},  # temporal plan graph
        Vector{Vector{Node{State}}},    # solution
        Dict{Symbol,Float64},           # final cost
    } where {State<:AbsState}

smoothing by temporal plan graph
"""
function smoothing(
    solution::Vector{Vector{Node{State}}},
    connect::Function,
    collide::Function;
    VERBOSE::Int64 = 0,
)::Tuple{
    Vector{Vector{Action{State}}},  # temporal plan graph
    Vector{Vector{Node{State}}},  # solution
    Dict{Symbol,Float64},  # final cost
} where {State<:AbsState}

    isnothing(solution) && return nothing

    solution_tmp = solution
    config_goal = map(v -> v.q, solution[end])
    cost_last = nothing
    sum_of_cost_last = Inf
    while true
        # 1. create temporal plan graph
        TPG = get_temporal_plan_graph(solution_tmp, collide, connect)
        # 2. sampling from temporal plan graph
        solution_tmp = get_greedy_solution(TPG, config_goal)
        cost = get_tpg_cost(TPG)
        if sum_of_cost_last >= cost[:sum_of_cost]
            return (TPG, solution_tmp, cost)
        else
            # 3. update solution
            VERBOSE > 0 && @info @sprintf("cost is updated: %f -> %f", cost_last, cost)
            cost_last = cost
            sum_of_cost_last = get(cost, :sum_of_cost)
        end
    end
end

function smoothing(solution::Nothing, args...; kwargs...)::Nothing
    nothing
end

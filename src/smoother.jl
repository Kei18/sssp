mutable struct Action{State<:AbsState}
    id::String
    from::Node{State}
    to::Node{State}
    t::Int64
    agent::Int64
    predecessors::Vector{Tuple{Int64, String}}
    successors::Vector{Tuple{Int64, String}}
end

function get_action_id(from::Node{State}, to::Node{State}, t::Int64)::String where State<:AbsState
    return string(from.id) * "_" * string(to.id) * "_" * string(t)
end

function get_temporal_plan_graph(
    solution::Vector{Vector{Node{State}}},
    collide::Function,
    connect::Function;
    skip_connection::Bool=true
    )::Vector{Vector{Action{State}}} where State<:AbsState

    N = length(solution[1])
    T = length(solution)

    # temporal plan graph
    TPG = [ Vector{Action{State}}() for i=1:N ]

    # type-1 dependency
    for i = 1:N
        v_last = solution[1][i]
        for (t, Q) in enumerate(solution[1:end])
            v_current = Q[i]

            if v_current.id != v_last.id
                action = Action(get_action_id(v_last, v_current, t), v_last, v_current, t, i,
                                Vector{Tuple{Int64, String}}(), Vector{Tuple{Int64, String}}())

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
                if j == i; continue; end

                # exclude ealier actions
                for action_other in filter(a -> a.t > action_self.t, TPG[j])

                    # create type-2 dependency
                    if collide(action_self.from.q, action_self.to.q,
                               action_other.from.q, action_other.to.q, i, j)
                        push!(action_other.predecessors, (action_self.agent, action_self.id))
                        break
                    end
                end
            end
        end
    end

    # remove redundant dependencies
    for i = 1:N
        for action in TPG[i]
            latest_actions = Dict{Int64, Action}()
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

    if skip_connection; try_skip_connection!(TPG, collide, connect); end
    return TPG
end

function try_skip_connection!(
    TPG::Vector{Vector{Action{State}}}, collide::Function, connect::Function
    ) where State<:AbsState

    N = length(TPG)
    get_action = (j, id) -> TPG[j][findfirst(a -> a.id == id, TPG[j])]

    for i = 1:N
        k = 1
        while k < length(TPG[i])
            k += 1
            a1 = TPG[i][k-1]
            a2 = TPG[i][k]

            # check connection
            if !connect(a1.from.q, a2.to.q, i; ignore_eps=true); continue; end

            # check dependencies
            if any([ j != i for (j, _) in a2.predecessors]); continue; end
            if any([ j != i for (j, _) in a1.successors]); continue; end

            # new action candidate
            a3 = Action(get_action_id(a1.from, a2.to, a1.t),
                        a1.from, a2.to, a1.t, a1.agent, a1.predecessors, a2.successors)

            # get causalities
            causal_actions = get_ancestors(a1, TPG)
            union!(causal_actions, get_descendants(a2, TPG))
            causal_actions = filter(val -> val[1] != i, causal_actions)

            # check collisions
            conflicted = false
            for j = 1:N
                if j == i; continue; end
                for a4 in filter(a -> !((j, a.id) in causal_actions), TPG[j])
                    if collide(a3.from.q, a3.to.q, a4.from.q, a4.to.q, i, j)
                        conflicted = true
                        break
                    end
                end
                if conflicted; continue; end
                # check last location
                if collide(a3.from.q, a3.to.q, TPG[j][end].to.q, TPG[j][end].to.q, i, j)
                    conflicted = true
                    break
                end
            end
            if conflicted; continue; end

            # update action orders
            deleteat!(TPG[i], k-1:k)
            insert!(TPG[i], k-1, a3)

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

function get_causal_actions(
    action::Action{State},
    TPG::Vector{Vector{Action{State}}},
    for_ancestors::Bool
    ) where State<:AbsState

    N = length(TPG)
    tables = [ Dict{String, Set{Tuple{Int64, String}}}() for i in 1:N ]
    function f(i, id)
        action = TPG[i][findfirst(a -> a.id == id, TPG[i])]
        if haskey(tables[i], id); return tables[i][id]; end
        causal_actions = Set{Tuple{Int64, String}}()
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
    ) where State<:AbsState
    return get_causal_actions(action, TPG, true)
end

function get_descendants(
    action::Action{State},
    TPG::Vector{Vector{Action{State}}},
    ) where State<:AbsState
    return get_causal_actions(action, TPG, false)
end

function get_greedy_solution(
    TPG::Vector{Vector{Action{State}}};
    config_goal::Union{Vector{State}, Nothing}=nothing
    )::Vector{Vector{Node{State}}} where State<:AbsState
    N = length(TPG)
    indexes = [ 1 for i = 1:N ]
    solution = [[ (length(arr) > 0 ? arr[1].from : Node(config_goal[i], 0, Vector{Int64}()))
                  for (i, arr) in enumerate(TPG) ]]

    while !all([k > length(TPG[i]) for (i, k) in enumerate(indexes)])
        config = []
        indexes_last = copy(indexes)
        for i = 1:N
            if length(TPG[i]) > 0
                action = TPG[i][min(indexes[i], length(TPG[i]))]
                if all([findfirst(a -> a.id == id, TPG[j]) < indexes_last[j]
                        for (j, id) in action.predecessors])
                    indexes[i] = min(indexes[i] + 1, length(TPG[i]) + 1)
                    push!(config, action.to)
                else
                    push!(config, action.from)
                end
            elseif config_goal != nothing
                push!(config, Node(config_goal[i], 0, Vector{Int64}()))
            end
        end
        push!(solution, config)
    end
    return solution
end

function get_tpg_cost(
    TPG::Vector{Vector{Action{State}}},
    func::Function=sum,
    ) where State<:AbsState

    N = length(TPG)
    cost_tables = [ Dict{String, Float64}() for i in 1:N ]

    function f(i, id)
        action = TPG[i][findfirst(a -> a.id == id, TPG[i])]
        c = dist(action.from.q, action.to.q)
        if !isempty(action.predecessors)
            c += maximum([(haskey(cost_tables[j], id_j) ? cost_tables[j][id_j] : f(j, id_j))
                          for (j, id_j) in action.predecessors ])
        end
        cost_tables[i][id] = c
        return c
    end

    return func([ (length(TPG[i]) > 0 ? f(i, TPG[i][end].id) : 0) for i in 1:N])
end

import Dates


function smoothing(
    solution::Vector{Vector{Node{State}}},
    collide::Function,
    connect::Function;
    cost_fn::Function=sum,
    skip_connection::Bool=true
    )::Tuple{
        Vector{Vector{Action{State}}}, Vector{Vector{Node{State}}}, Float64
    } where State<:AbsState

    solution_tmp = solution
    config_goal = map(path -> path[end].q, solution)
    cost_last = 0
    while true
        TPG = get_temporal_plan_graph(solution_tmp, collide, connect; skip_connection=skip_connection)
        solution_tmp = get_greedy_solution(TPG; config_goal=config_goal)
        cost = get_tpg_cost(TPG, cost_fn)
        if cost_last == cost
            return (TPG, solution_tmp, cost)
        else
            cost_last = cost
        end
    end
end

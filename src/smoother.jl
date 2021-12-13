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
                    Q_from = [action_other.from, action_self.from]
                    Q_to = [action_other.to, action_self.to]
                    if collide(Q_from, Q_to)
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
            if !connect(a1.from.q, a2.to.q, i); continue; end

            # check dependencies
            if any([ j != i for (j, _) in a2.predecessors]); continue; end
            if any([ j != i for (j, _) in a1.successors]); continue; end

            # new action candidate
            a3 = Action(get_action_id(a1.from, a2.to, a1.t),
                        a1.from, a2.to, a1.t, a1.agent, a1.predecessors, a2.successors)

            # check collisions
            conflicted = false
            for j = 1:N
                if j == i; continue; end
                for a4 in filter(a -> a1.t < a.t < a2.t, TPG[j])
                    Q_from = [a3.from, a4.from]
                    Q_to = [a3.to, a4.to]
                    if collide(Q_from, Q_to)
                        conflicted = true;
                        break
                    end
                end
                if conflicted; continue; end
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

function get_greedy_solution(
    TPG::Vector{Vector{Action{State}}}
    )::Vector{Vector{Node{State}}} where State<:AbsState
    N = length(TPG)
    indexes = [ 1 for i = 1:N ]
    solution = [[arr[1].from for arr in TPG]]

    while !all([k > length(TPG[i]) for (i, k) in enumerate(indexes)])
        config = []
        indexes_last = copy(indexes)
        for i = 1:N
            action = TPG[i][min(indexes[i], length(TPG[i]))]
            if all([findfirst(a -> a.id == id, TPG[j]) < indexes_last[j]
                    for (j, id) in action.predecessors])
                indexes[i] = min(indexes[i] + 1, length(TPG[i]) + 1)
                push!(config, action.to)
            else
                push!(config, action.from)
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


    return func([f(i, TPG[i][end].id) for i in 1:N])
end


function smoothing(
    solution::Vector{Vector{Node{State}}},
    collide::Function,
    connect::Function
    )::Vector{Vector{Node{State}}} where State<:AbsState

    TPG = get_temporal_plan_graph(solution, collide, connect)
    return get_greedy_solution(TPG)
end

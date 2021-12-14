function refine!(
    # problem instances
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function,
    # data from past iteration
    solution::Vector{Vector{Node{State}}},
    V::Vector{Vector{Node{State}}},
    # specify search details
    h_func::Function,
    g_func::Function,
    random_walk::Function,
    get_sample_nums::Function = (k::Int64) -> k + 3;
    # other parameters
    agents_refine::Vector{Int64}=[rand(1:length(config_init))],
    MAX_ITER::Int64 = 3,
    MAX_LOOP_CNT::Int64 = 100000,
    VERBOSE::Int64 = 1,
    VERBOSE_LOOP_CNT::Int64 = 100,
    TIME_LIMIT::Union{Nothing, Real}=nothing
    ) where State<:AbsState

    # for timeout
    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    # number of agents
    N = length(config_init)

    # makespan
    T = length(solution)

    # setup refinement
    agents_fixed = filter(i -> !(i in agents_refine), 1:N)
    if VERBOSE > 0; println("\trefined agents:", agents_refine); end

    function get_Q_id(Q::Vector{Node{State}}, t::Int64, next::Int64)::String
        return join([v.id for v in Q], "-") * "_" * string(t) * "_" * string(next)
    end

    # initial search node
    Q_init = solution[1]
    S_init = SuperNode(Q=Q_init, next=0, id=get_Q_id(Q_init, 1, 0), h=h_func(Q_init))

    # verbose
    print_progress! = (S::SuperNode{State}, k::Int64, loop_cnt::Int64; force::Bool=false) -> begin
        if VERBOSE == 0 || (!force && (loop_cnt % VERBOSE_LOOP_CNT != 0)); return; end
        @printf("\r\t%6.4f sec, iteration: %02d, explored node: %08d, ", elapsed(), k, loop_cnt)
        @printf("f: %.2f, g: %.2f, h: %.2f, depth: %04d", S.f, S.g, S.f, S.depth)
    end

    # iteration
    for k = 1:MAX_ITER

        # check timeout
        if timeover(); break; end

        # open list
        OPEN = PriorityQueue{SuperNode{State}, Float64}()

        # discovered list to avoid duplication
        VISITED = Dict{String, SuperNode{State}}()

        # setup initail node
        enqueue!(OPEN, S_init, S_init.f)
        VISITED[S_init.id] = S_init

        loop_cnt = 0
        while !isempty(OPEN) && !timeover()
            loop_cnt += 1

            # pop
            S = dequeue!(OPEN)

            # check goal
            if check_goal(S.Q)
                print_progress!(S, k, loop_cnt, force=true)
                if VERBOSE > 0; @printf("\n\t%6.4f sec: found solution\n", elapsed()) end
                return backtrack(S, VISITED)
            end

            if S.next == 0
                # update for fixed agents

                # case-1, skip
                Q_new_id = get_Q_id(S.Q, S.t, 1)
                if !haskey(VISITED, Q_new_id)  # collision check is unnecessary
                    S_new = SuperNode(
                        Q=S.Q,
                        t=S.t,
                        next=1,
                        id=Q_new_id,
                        parent_id=S.id,
                        h=h_func(S.Q),
                        g=S.g,
                        depth=S.depth+1
                    )
                    enqueue!(OPEN, S_new, S_new.f)
                    VISITED[S_new.id] = S_new
                end

                # case-2: update fixed agents' states
                Q_new = copy(S.Q)
                t = min(S.t+1, T)
                for k in agents_fixed; Q_new[k] = solution[t][k]; end
                Q_new_id = get_Q_id(Q_new, t, 1)
                if !haskey(VISITED, Q_new_id) && !collide(S.Q, Q_new)
                    # insert
                    S_new = SuperNode(
                        Q=Q_new,
                        t=t,
                        next=1,
                        id=Q_new_id,
                        parent_id=S.id,
                        h=h_func(Q_new),
                        g=S.g+g_func(S.Q, Q_new),
                        depth=S.depth+1
                    )
                    enqueue!(OPEN, S_new, S_new.f)
                    VISITED[S_new.id] = S_new
                end
            else
                # update for refine agents

                i = agents_refine[S.next]
                j = (S.next < length(agents_refine)) ? S.next + 1 : 0
                v = S.Q[i]

                # explore new states
                expand!(V[i], v.q, i, get_sample_nums(k), connect, random_walk)

                # expand search node
                for p_id in vcat(v.neighbors, [v.id])

                    # create new configuration
                    p = V[i][p_id]
                    Q = copy(S.Q)
                    Q[i] = p

                    # check duplication and collision
                    Q_id = get_Q_id(Q, S.t, j)
                    if haskey(VISITED, Q_id) || collide(S.Q, Q); continue; end

                    # create new search node
                    S_new = SuperNode(
                        Q=Q,
                        t=S.t,
                        next=j,
                        id=Q_id,
                        parent_id=S.id,
                        h=h_func(Q),
                        g=S.g+g_func(S.Q, Q),
                        depth=S.depth+1
                    )

                    # insert
                    enqueue!(OPEN, S_new, S_new.f)
                    VISITED[S_new.id] = S_new
                end
            end

            print_progress!(S, k, loop_cnt, force=isempty(OPEN))
        end

        if VERBOSE > 0; println(); end
    end

    if VERBOSE > 0; @printf("\n\t%6.4f sec: failed to find solution\n", elapsed()) end
    return nothing
end

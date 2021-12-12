import DataStructures: PriorityQueue, enqueue!, dequeue!
import Printf: @printf, @sprintf

abstract type AbsState end
abstract type StatePoint <: AbsState end

mutable struct Node{State<:AbsState}
    q::State
    id::Int64
    neighbors::Vector{Int64}
end

mutable struct SuperNode{State<:AbsState}
    Q::Vector{Node{State}}
    next::Int64
    id::String
    parent_id::Union{Nothing, String}
    f::Float64
    g::Float64
    h::Float64
    depth::Int64
end

function search(
    # problem instances
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function,
    # specify search details
    h_func::Function,
    g_func::Function,
    random_walk::Function,
    get_sample_nums::Function = (k::Int64) -> k + 3;
    # other parameters
    MAX_ITER::Int64 = 3,
    MAX_LOOP_CNT::Int64 = 100000,
    VERBOSE::Int64 = 1,
    VERBOSE_LOOP_CNT::Int64 = 100
    ) where State<:AbsState

    # number of agents
    N = length(config_init)

    # graph for respective agents, insert start and goal
    V = [[Node{State}(q_init, 1, []), Node{State}(q_goal, 2, [])]
         for (q_init, q_goal) in zip(config_init, config_goal)]

    # initial configuration
    Q_init = [ V[i][1] for i = 1:N ]

    # to generate id of search nodes
    get_S_id(Q::Vector{Node{State}}, next::Int64) = join([v.id for v in Q], "-") * "_" * string(next)

    # initial search node
    g = 0
    h = h_func(Q_init)
    f = g + h
    S_init = SuperNode{State}(Q_init, 1, get_S_id(Q_init, 1), nothing, f, g, h, 1)

    # verbose
    print_progress! = (S::SuperNode{State}, k::Int64, loop_cnt::Int64; force::Bool=false) -> begin
        if VERBOSE < 0 || (!force && (loop_cnt % VERBOSE_LOOP_CNT != 0)); return; end
        @printf("\riteration: %02d, explored node: %08d, f: %.2f, g: %.2f, h: %.2f, depth: %04d",
                k, loop_cnt, S.f, S.g, S.f, S.depth)
    end

    # iteration
    for k = 1:MAX_ITER

        # open list
        OPEN = PriorityQueue{SuperNode{State}, Float64}()

        # discovered list to avoid duplication
        VISITED = Dict{String, SuperNode{State}}()

        # setup initail node
        enqueue!(OPEN, S_init, S_init.f)
        VISITED[S_init.id] = S_init

        # main loop
        loop_cnt = 0
        while !isempty(OPEN)
            loop_cnt += 1
            if loop_cnt > MAX_LOOP_CNT; break; end

            # pop
            S = dequeue!(OPEN)

            # check goal
            if check_goal(S.Q)
                print_progress!(S, k, loop_cnt, force=true)
                if VERBOSE > 0
                    println()
                    println("found solution")
                end
                return (backtrack(S, VISITED), V)
            end

            i = S.next
            v = S.Q[i]
            expand!(V[i], v.q, i, get_sample_nums(k), connect, random_walk)
            j = (i < N) ? i + 1 : 1

            # expand search node
            for p_id in vcat(v.neighbors, [v.id])

                # create new configuration
                p = V[i][p_id]
                Q = copy(S.Q)
                Q[i] = p
                new_id = get_S_id(Q, j)

                # check duplication and collision
                if haskey(VISITED, new_id) || collide(S.Q, Q); continue; end

                # create new search node
                g = S.g + g_func(S.Q, Q)
                h = h_func(Q)
                f = g + h
                S_new = SuperNode{State}(Q, j, new_id, S.id, f, g, h, S.depth+1)

                # insert
                enqueue!(OPEN, S_new, S_new.f)
                VISITED[S_new.id] = S_new
            end

            print_progress!(S, k, loop_cnt, force=isempty(OPEN))
        end

        if VERBOSE > 0; println(); end
    end

    if VERBOSE > 0; println(); end
    println("failed to find solution")
    return (nothing, V)
end


function expand!(
    V::Vector{Node{State}},
    q::State,
    i::Int64,
    K::Int64,
    connect::Function,
    random_walk::Function
    ) where State <: AbsState
    for j = 1:K
        # sample one point
        q_new = random_walk(q)

        # check connection
        if !connect(q, q_new, i); continue; end

        # identify candidate neighbors
        C_v_u = filter(v -> connect(v.q, q_new, i), V)
        C_u_v = filter(v -> connect(q_new, v.q, i), V)

        # check the number of neighbors in candidates
        if length(C_v_u) >= K || length(C_u_v) >= K; continue; end

        # add vertex and edges
        u = Node(q_new, length(V) + 1, Vector{Int64}())
        push!(V, u)
        for v in C_v_u; push!(v.neighbors, u.id); end
        for v in C_u_v; push!(u.neighbors, v.id); end
    end
end


function backtrack(
    S_fin::SuperNode{State},
    VISITED::Dict{String, SuperNode{State}}
    )::Vector{Vector{State}} where State<:AbsState

    S = S_fin
    solution = []
    while S.parent_id != nothing
        pushfirst!(solution, map(v -> v.q, S.Q))
        S = VISITED[S.parent_id]
    end
    pushfirst!(solution, map(v -> v.q, S.Q))
    return solution
end

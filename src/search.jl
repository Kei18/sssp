abstract type AbsState end
abstract type StatePoint <: AbsState end

mutable struct Node{State<:AbsState}
    q::State
    id::Int64
    neighbors::Vector{Int64}
end

# to generate id of search nodes
function get_Q_id(
    Q::Vector{Node{State}},
    t::Int64,
    next::Int64,
)::String where {State<:AbsState}
    return @sprintf("%s_%d_%d", join([v.id for v in Q], "-"), t, next)
end

@kwdef mutable struct SuperNode{State<:AbsState}
    Q::Vector{Node{State}}  # set of search nodes
    next::Int64  # next agent, 0 -> fixed agents
    id::String = get_Q_id(Q, 1, next)
    parent_id::Union{Nothing,String} = nothing  # parent node
    g::Float64 = 0.0  # g-value
    h::Float64 = 0.0  # h-value
    f::Float64 = g + h  # f-value
    depth::Int64 = 1  # depth
    t::Int64 = 1 # used in refinement phase
end

function search!(
    # problem instances & search details
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function,
    h_func::Function,
    g_func::Function,
    random_walk::Function,
    get_sample_nums::Function = (k::Int64) -> k + 3;

    # data from past iteration
    current_solution::Union{Nothing,Vector{Vector{Node{State}}}} = nothing,
    # might be updated
    roadmaps::Vector{Vector{Node{State}}} = (map(
        e -> [Node{State}(e[1], 1, []), Node{State}(e[2], 2, [])],
        zip(config_init, config_goal),
    )),
    NUM_REFINE_AGENTS::Int64 = 1,
    AGENTS_REFINE::Vector{Int64} = (
        current_solution == nothing ? collect(1:length(config_init))  # initial solution
        : randperm(length(config_init))[1:NUM_REFINE_AGENTS]  # refinement
    ),

    # other search parameters
    MAX_ITER::Int64 = 1000,
    MAX_LOOP_CNT::Int64 = 100000,
    VERBOSE::Int64 = 1,
    VERBOSE_LOOP_CNT::Int64 = 100,
    TIME_LIMIT::Union{Nothing,Real} = nothing,
)::Tuple{
    Union{Nothing,Vector{Vector{Node{State}}}},  # solution
    Vector{Vector{Node{State}}},  # roadmap
} where {State<:AbsState}
    """get initial solution / refinement"""

    # ------------------------------
    # step 0. define utilities
    # ------------------------------

    # timeout
    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    # number of agents
    N = length(config_init)

    # for refinement
    is_refine = current_solution != nothing
    T = is_refine ? length(current_solution) : 0
    AGENTS_FIXED = filter(i -> !(i in AGENTS_REFINE), 1:N)
    if VERBOSE > 0 && is_refine
        println("\trefined agents:", AGENTS_REFINE)
    end

    # verbose
    print_progress! =
        (S::SuperNode{State}, k::Int64, loop_cnt::Int64; force::Bool = false) -> begin
            if VERBOSE == 0 || (!force && (loop_cnt % VERBOSE_LOOP_CNT != 0))
                return
            end
            @printf(
                "\r\t%6.4f sec, iteration: %02d, explored node: %08d, ",
                elapsed(),
                k,
                loop_cnt
            )
            @printf("f: %.2f, g: %.2f, h: %.2f, depth: %04d", S.f, S.g, S.f, S.depth)
        end

    # ------------------------------
    # step 1. setup initial node
    # ------------------------------

    # initial configuration
    Q_init = is_refine ? current_solution[1] : [roadmaps[i][1] for i = 1:N]

    # initial search node
    S_init = SuperNode(
        Q = Q_init,
        next = (is_refine ? 0 : 1),  # refinement or initial solution
        id = get_Q_id(Q_init, 1, 0),
        h = h_func(Q_init),
    )

    # ------------------------------
    # step 2. start search
    # ------------------------------

    # iteration
    for k = 1:MAX_ITER

        # check timeout
        if timeover()
            break
        end

        # open list
        OPEN = PriorityQueue{SuperNode{State},Float64}()

        # discovered list to avoid duplication
        VISITED = Dict{String,SuperNode{State}}()

        # setup initail node
        enqueue!(OPEN, S_init, S_init.f)
        VISITED[S_init.id] = S_init

        loop_cnt = 0
        while !isempty(OPEN) && !timeover()
            loop_cnt += 1
            if loop_cnt > MAX_LOOP_CNT
                break
            end

            # pop
            S = dequeue!(OPEN)

            # check goal
            if check_goal(S.Q)
                print_progress!(S, k, loop_cnt, force = true)
                if VERBOSE > 0
                    @printf("\n\t%6.4f sec: found solution\n", elapsed())
                end
                return (backtrack(S, VISITED), roadmaps)
            end

            if S.next > 0
                # initial search or update for refine agents
                i = AGENTS_REFINE[S.next]
                j = (
                    # refinement or initial solution
                    S.next < length(AGENTS_REFINE) ? S.next + 1 : (is_refine ? 0 : 1)
                )

                v = S.Q[i]

                # explore new states
                expand!(roadmaps[i], v.q, i, get_sample_nums(k), connect, random_walk)

                # expand search node
                for p_id in vcat(v.neighbors, [v.id])

                    # create new configuration
                    p = roadmaps[i][p_id]
                    Q = copy(S.Q)
                    Q[i] = p

                    # check duplication and collision
                    Q_id = get_Q_id(Q, S.t, j)
                    if haskey(VISITED, Q_id) || collide(S.Q, Q)
                        continue
                    end

                    # create new search node
                    S_new = SuperNode(
                        Q = Q,
                        t = S.t,
                        next = j,
                        id = Q_id,
                        parent_id = S.id,
                        h = h_func(Q),
                        g = S.g + g_func(S.Q, Q),
                        depth = S.depth + 1,
                    )

                    # insert
                    enqueue!(OPEN, S_new, S_new.f)
                    VISITED[S_new.id] = S_new
                end

            else
                # update for fixed agents, used only in the refinement phase
                @assert is_refine

                # case-1, skip
                Q_new_id = get_Q_id(S.Q, S.t, 1)
                if !haskey(VISITED, Q_new_id)  # collision check is unnecessary
                    S_new = SuperNode(
                        Q = S.Q,
                        t = S.t,
                        next = 1,
                        id = Q_new_id,
                        parent_id = S.id,
                        h = h_func(S.Q),
                        g = S.g,
                        depth = S.depth + 1,
                    )
                    enqueue!(OPEN, S_new, S_new.f)
                    VISITED[S_new.id] = S_new
                end

                # case-2: update fixed agents' states
                Q_new = copy(S.Q)
                t = min(S.t + 1, T)
                for k in AGENTS_FIXED
                    Q_new[k] = current_solution[t][k]
                end
                Q_new_id = get_Q_id(Q_new, t, 1)
                if !haskey(VISITED, Q_new_id) && !collide(S.Q, Q_new)
                    # insert
                    S_new = SuperNode(
                        Q = Q_new,
                        t = t,
                        next = 1,
                        id = Q_new_id,
                        parent_id = S.id,
                        h = h_func(Q_new),
                        g = S.g + g_func(S.Q, Q_new),
                        depth = S.depth + 1,
                    )
                    enqueue!(OPEN, S_new, S_new.f)
                    VISITED[S_new.id] = S_new
                end
            end

            print_progress!(S, k, loop_cnt, force = isempty(OPEN))
        end

        if VERBOSE > 0
            println()
        end
    end

    if VERBOSE > 0
        @printf("\n\t%6.4f sec: failed to find solution\n", elapsed())
    end
    return (nothing, roadmaps)
end


function expand!(
    roadmap::Vector{Node{State}},
    q::State,
    i::Int64,
    K::Int64,
    connect::Function,
    random_walk::Function,
)::Nothing where {State<:AbsState}
    """expand roadmap by random walk"""

    for j = 1:K
        # sample one point
        q_new = random_walk(q)

        # check connection
        if !connect(q, q_new, i)
            continue
        end

        # identify candidate neighbors
        C_v_u = filter(v -> connect(v.q, q_new, i), roadmap)
        C_u_v = filter(v -> connect(q_new, v.q, i), roadmap)

        # check the number of neighbors in candidates
        if length(C_v_u) >= K || length(C_u_v) >= K
            continue
        end

        # add vertex and edges
        u = Node(q_new, length(roadmap) + 1, Vector{Int64}())
        push!(roadmap, u)
        for v in C_v_u
            push!(v.neighbors, u.id)
        end
        for v in C_u_v
            push!(u.neighbors, v.id)
        end
    end
end


function backtrack(
    S_fin::SuperNode{State},
    VISITED::Dict{String,SuperNode{State}},
)::Vector{Vector{Node{State}}} where {State<:AbsState}
    """generate solution from the final search node"""

    S = S_fin
    solution = Vector{Vector{Node{State}}}()
    while S.parent_id != nothing
        pushfirst!(solution, S.Q)
        S = VISITED[S.parent_id]
    end
    pushfirst!(solution, S.Q)
    return solution
end

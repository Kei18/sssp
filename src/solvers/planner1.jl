import DataStructures: PriorityQueue, enqueue!, dequeue!
import Printf: @printf, @sprintf
import Base: @kwdef
import Random: randperm

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

# to generate id of search nodes
function get_Q_id(
    Q::Vector{Node{State}},
    t::Int64,
    next::Int64,
)::String where {State<:AbsState}
    return @sprintf("%s_%d_%d", join([v.id for v in Q], "-"), t, next)
end


function planner1(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function;
    g_func::Function = gen_g_func(greedy = true),
    max_iteration::Union{Nothing,Int64} = nothing,
    num_vertex_expansion::Int64 = 10,
    init_min_dist_thread::Float64 = 0.1,
    decreasing_rate_min_dist_thread::Float64 = 0.99,
    steering_depth::Int64 = 2,
    max_makespan::Union{Nothing,Int64} = nothing,
    TIME_LIMIT::Union{Nothing,Real} = 30,
    VERBOSE::Int64 = 0,
)::Tuple{
    Union{Nothing,Vector{Vector{Node{State}}}},  # solution
    Vector{Vector{Node{State}}},  # roadmap
} where {State<:AbsState}
    """get initial solution / refinement"""

    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    # number of agents
    N = length(config_init)

    roadmaps = map(
        i -> [Node{State}(config_init[i], 1, []), Node{State}(config_goal[i], 2, [])],
        1:N,
    )
    for i = 1:N
        if connect(config_init[i], config_goal[i], i)
            push!(roadmaps[i][1].neighbors, 2)
            push!(roadmaps[i][2].neighbors, 1)
        end
    end

    sampler = MRMP.gen_uniform_sampling(config_init[1])

    # verbose
    print_progress! =
        (S::SuperNode{State}, k::Int64, loop_cnt::Int64; force::Bool = false) -> begin
            (VERBOSE == 0 || (!force && (loop_cnt % 100 != 0))) && return
            @printf(
                "\r\t%6.4f sec, iteration: %02d, explored node: %08d, f: %.2f, g: %.2f, h: %.2f, depth: %04d",
                elapsed(),
                k,
                loop_cnt,
                S.f,
                S.g,
                S.f,
                S.depth
            )
        end

    h_func(Q::Vector{Node{State}}) = sum(map(i -> dist(Q[i].q, config_goal[i]), 1:N)) / N

    # initial configuration
    Q_init = [roadmaps[i][1] for i = 1:N]

    # initial search node
    S_init =
        SuperNode(Q = Q_init, next = 1, id = get_Q_id(Q_init, 1, 0), h = h_func(Q_init))

    # iteration
    k = 0
    while !timeover() && (isnothing(max_iteration) || k < max_iteration)
        k += 1

        # open list
        OPEN = PriorityQueue{SuperNode{State},Float64}()

        # discovered list to avoid duplication
        VISITED = Dict{String,SuperNode{State}}()

        # already expanded in this iteration
        EXPANDED = [Dict{Int64,Bool}() for i = 1:N]

        min_dist_thread = init_min_dist_thread * (decreasing_rate_min_dist_thread^(k - 1))

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
                print_progress!(S, k, loop_cnt, force = true)
                VERBOSE > 0 && @printf("\n\t%6.4f sec: found solution\n", elapsed())
                return (backtrack(S, VISITED), roadmaps)
            end

            if S.next > 0
                # initial search or update for refine agents
                i = S.next
                j = mod1(S.next + 1, N)

                v = S.Q[i]

                # explore new states
                if !get(EXPANDED[i], v.id, false)
                    EXPANDED[i][v.id] = true

                    for _ = 1:num_vertex_expansion
                        # steering
                        q_new = MRMP.steering(v.q, sampler(), connect, i, steering_depth)

                        # identify neighbors
                        C_v_u = filter(v -> connect(v.q, q_new, i), roadmaps[i])
                        C_u_v = filter(v -> connect(q_new, v.q, i), roadmaps[i])

                        # check space-filling metric
                        if isempty(C_v_u) ||
                           minimum(map(v -> dist(v.q, q_new), C_v_u)) > min_dist_thread
                            # add vertex and edges
                            u = Node(q_new, length(roadmaps[i]) + 1, Vector{Int64}())
                            push!(roadmaps[i], u)
                            # update neighbors
                            foreach(v -> push!(v.neighbors, u.id), C_v_u)
                            foreach(v -> push!(u.neighbors, v.id), C_u_v)
                        end
                    end
                end

                # expand search node
                for p_id in vcat(v.neighbors, [v.id])

                    # create new configuration
                    p = roadmaps[i][p_id]
                    Q = copy(S.Q)
                    Q[i] = p

                    # check duplication and collision
                    Q_id = get_Q_id(Q, S.t, j)
                    if haskey(VISITED, Q_id) || collide(S.Q, p.q, i)
                        continue
                    end

                    # check max makespan
                    !isnothing(max_makespan) && max_makespan * N < S.depth + 1 && continue

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
            end

            print_progress!(S, k, loop_cnt, force = isempty(OPEN))
        end

        VERBOSE > 0 && println()
    end

    VERBOSE > 0 && @printf("\n\t%6.4f sec: failed to find solution\n", elapsed())
    return (nothing, roadmaps)
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

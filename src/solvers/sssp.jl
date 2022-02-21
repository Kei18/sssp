"""implementation of Simultaneous Sampling-and-Search Planning (SSSP)"""
module LibSSSP
export SSSP

import Printf: @sprintf, @printf
import Base: @kwdef
import ...MRMP: AbsState, Node, now, elapsed_sec, gen_uniform_sampling, get_mid_status, dist
import DataStructures: PriorityQueue, enqueue!, dequeue!
import ..Solvers: gen_g_func, get_distance_tables, get_distance_table

@kwdef mutable struct SuperNode{State<:AbsState}
    Q::Vector{Node{State}}  # set of search nodes
    next::Int64  # next agent, 0 -> fixed agents
    id::String = get_Q_id(Q, next)
    parent_id::Union{Nothing,String} = nothing  # parent node
    g::Float64 = 0.0  # g-value
    h::Float64 = 0.0  # h-value
    f::Float64 = g + h  # f-value
    depth::Int64 = 1  # depth
end

"""
    SSSP(
        config_init::Vector{State},
        config_goal::Vector{State},
        connect::Function,
        collide::Function,
        check_goal::Function;
        g_func::Function = gen_g_func(greedy = true),
        steering_depth::Int64 = 2,
        num_vertex_expansion::Int64 = 10,
        init_min_dist_thread::Float64 = 0.1,
        decreasing_rate_min_dist_thread::Float64 = 0.99,
        epsilon::Union{Float64,Nothing} = nothing,
        TIME_LIMIT::Union{Nothing,Real} = 30,
        VERBOSE::Int64 = 0,

        use_random_h_func::Bool = false,         # for ablation study
        no_roadmap_at_beginning::Bool = false,   # for ablation study

        no_fast_collision_check::Bool = false,   # use 'slow' collision checker
    )::Tuple{
        Union{Nothing,Vector{Vector{Node{State}}}},  # solution
        Vector{Vector{Node{State}}},  # roadmap
    } where {State<:AbsState}

implementation of SSSP
"""
function SSSP(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function;
    g_func::Function = gen_g_func(greedy = true),
    steering_depth::Int64 = 2,
    num_vertex_expansion::Int64 = 10,
    init_min_dist_thread::Float64 = 0.1,
    decreasing_rate_min_dist_thread::Float64 = 0.99,
    epsilon::Union{Float64,Nothing} = nothing,
    TIME_LIMIT::Union{Nothing,Real} = 30,
    VERBOSE::Int64 = 0,

    # for ablation study
    use_random_h_func::Bool = false,
    no_roadmap_at_beginning::Bool = false,

    # just for fairness of experiments
    no_fast_collision_check::Bool = false,
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

    # define sampler
    sampler = gen_uniform_sampling(config_init[1])

    # used in steering
    conn(q_from::State, q_to::State, i::Int64) = begin
        (isnothing(epsilon) || dist(q_from, q_to) <= epsilon) && connect(q_from, q_to, i)
    end

    # get initial roadmap by RRT-connect
    roadmaps = (
        no_roadmap_at_beginning ?
        map(
            i -> begin
                v_init = Node{State}(config_init[i], 1, [])
                v_goal = Node{State}(config_goal[i], 2, [])
                conn(v_init.q, v_goal.q, i) && push!(v_init.neighbors, v_goal.id)
                conn(v_goal.q, v_init.q, i) && push!(v_goal.neighbors, v_init.id)
                [v_init, v_goal]
            end,
            1:N,
        ) :
        gen_RRT_connect_roadmaps(
            config_init,
            config_goal,
            connect;
            steering_depth = steering_depth,
            epsilon = epsilon,
            TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
        )
    )
    if isnothing(roadmaps)
        VERBOSE > 0 &&
            @info @sprintf("\t%6.4f sec: failed to construct initial roadmaps\n", elapsed())
        return (nothing, map(i -> Vector{Node{State}}(), 1:N))
    end

    VERBOSE > 0 && !no_roadmap_at_beginning && @info ("\tdone, setup initial roadmaps")

    # setup distance tables
    distance_tables = get_distance_tables(roadmaps)

    # verbose
    print_progress! =
        (S::SuperNode{State}, loop_cnt::Int64; force::Bool = false) -> begin
            (VERBOSE < 1 || (!force && (loop_cnt % 100 != 0))) && return
            @printf(
                "\r\t\t%6.4f sec, explored node: %08d, f: %.4f, depth: %04d",
                elapsed(),
                loop_cnt,
                S.f,
                S.depth
            )
        end

    # setup heuristic function
    h_func(Q::Vector{Node{State}}) =
        (use_random_h_func ? rand() : sum(map(i -> distance_tables[i][Q[i].id], 1:N)) / N)

    # initial configuration
    Q_init = [roadmaps[i][1] for i = 1:N]

    # initial search node
    S_init = SuperNode(Q = Q_init, next = 1, id = get_Q_id(Q_init, 0), h = h_func(Q_init))

    k = 0
    while !timeover()
        k += 1

        # open list
        OPEN = PriorityQueue{SuperNode{State},Float64}()

        # discovered list to avoid duplication
        VISITED = Dict{String,SuperNode{State}}()

        # threshold of space-filling metric
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
                print_progress!(S, loop_cnt, force = true)
                VERBOSE > 0 && @info @sprintf("\n\t%6.4f sec: found solution\n", elapsed())
                return (backtrack(S, VISITED), roadmaps)
            end

            # initial search or update for refine agents
            i = S.next
            j = mod1(S.next + 1, N)

            v = S.Q[i]

            # explore new states
            expand!(
                (q_from::State, q_to::State) -> conn(q_from, q_to, i),
                sampler,
                v,
                roadmaps[i],
                min_dist_thread,
                num_vertex_expansion,
                steering_depth,
            ) && (distance_tables[i] = get_distance_table(roadmaps[i]))

            # expand search node
            for p_id in vcat(v.neighbors, v.id)

                # create new configuration
                p = roadmaps[i][p_id]
                Q = copy(S.Q)
                Q[i] = p

                # check duplication and collision
                Q_id = get_Q_id(Q, j)
                haskey(VISITED, Q_id) && continue
                !no_fast_collision_check && collide(S.Q, p.q, i) && continue
                no_fast_collision_check && collide(S.Q, Q) && continue

                # create new search node
                S_new = SuperNode(
                    Q = Q,
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
            print_progress!(S, loop_cnt, force = isempty(OPEN))
        end

        VERBOSE > 1 && println()
    end

    VERBOSE > 0 && @info @sprintf("\t%6.4f sec: failed to find solution\n", elapsed())
    return (nothing, roadmaps)
end

"""implementation of vertex expansion"""
function expand!(
    connect::Function,
    sampler::Function,
    v_from::Node{State},
    roadmap::Vector{Node{State}},
    min_dist_thread::Float64,
    num_vertex_expansion::Int64,
    steering_depth::Int64,
)::Bool where {State<:AbsState}

    updated = false
    for _ = 1:num_vertex_expansion
        # steering
        q_new = steering(connect, sampler(), v_from.q, steering_depth)
        # check space-filling metric
        if minimum(v -> dist(v.q, q_new), roadmap) > min_dist_thread
            # add vertex and edges
            u = Node(
                q_new,
                length(roadmap) + 1,
                map(v -> v.id, filter(v -> connect(q_new, v.q), roadmap)),
            )
            push!(roadmap, u)
            # update neighbors
            foreach(
                v -> push!(v.neighbors, u.id),
                filter(v -> connect(v.q, q_new), roadmap),
            )
            updated = true
        end
    end
    return updated
end

function steering(
    connect::Function,
    q_rand::State,
    q_near::State,
    steering_depth::Int64,
)::State where {State<:AbsState}

    q_h = q_rand  # probably unsafe -> eventually safe
    if !connect(q_near, q_h)
        # binary search
        q_l = q_near  # safe
        for _ = 1:steering_depth
            q = get_mid_status(q_l, q_h)
            if connect(q_near, q)
                q_l = q
            else
                q_h = q
            end
        end
        q_h = q_l
    end
    return q_h
end

"""generate initial roadmaps via RRT-Connect"""
function gen_RRT_connect_roadmaps(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function;
    steering_depth::Int64 = 2,
    epsilon::Union{Float64,Nothing} = 0.2,
    TIME_LIMIT::Union{Nothing,Real} = 30,
)::Union{Nothing,Vector{Vector{Node{State}}}} where {State<:AbsState}

    # utilities for timeout
    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    N = length(config_init)
    roadmaps = Vector{Vector{Node{State}}}()
    for i = 1:N
        rmp = gen_RRT_connect_roadmap(
            i,
            config_init[i],
            config_goal[i],
            connect;
            steering_depth = steering_depth,
            epsilon = epsilon,
            TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
        )
        isnothing(rmp) && return nothing
        push!(roadmaps, rmp)
    end
    return roadmaps
end

function gen_RRT_connect_roadmap(
    i::Int64,
    q_init::State,
    q_goal::State,
    connect::Function;
    steering_depth::Int64 = 2,
    epsilon::Union{Float64,Nothing} = 0.2,
    TIME_LIMIT::Union{Nothing,Real} = 30,
)::Union{Nothing,Vector{Node{State}}} where {State<:AbsState}

    # utilities for timeout
    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    # define sampler
    sampler = gen_uniform_sampling(q_init)

    # re-define connect function
    conn(q_from::State, q_to::State)::Bool = begin
        # check distance
        !(isnothing(epsilon) || dist(q_from, q_to) <= epsilon) && return false

        !connect(q_from, q_to, i) && return false

        return true
    end

    conn(q::State)::Bool = connect(q, i)

    # special case
    if conn(q_init, q_goal)
        v_init = Node(q_init, 1, [2])
        v_goal = Node(q_goal, 2, Vector{Int64}())
        conn(q_goal, q_init) && push!(v_goal.neighbors, v_init.id)
        return [v_init, v_goal]
    end

    # store all samples
    V1 = [q_init]
    V2 = [q_goal]

    V = V1

    # main loop
    iter = 0
    while !timeover()
        iter += 1
        from_start = (iter % 2 == 1)

        (flg, q_new) = extend!(conn, sampler(), V1, from_start, steering_depth)
        if flg != :trapped &&
           extend!(conn, q_new, V2, !from_start, steering_depth)[1] == :reached
            # fix tree
            if from_start
                V1, V2 = V2, V1
            end
            # create roadmap, insert vertices
            roadmap = [Node{State}(q_init, 1, []), Node{State}(q_goal, 2, [])]
            foreach(q -> push!(roadmap, Node{State}(q, length(roadmap) + 1, [])), V1[2:end])
            foreach(q -> push!(roadmap, Node{State}(q, length(roadmap) + 1, [])), V2[2:end])
            # create roadmap, insert edges
            for (k, v_from) in enumerate(roadmap)
                for (l, v_to) in enumerate(roadmap)
                    if k != l && conn(v_from.q, v_to.q)
                        push!(v_from.neighbors, v_to.id)
                    end
                end
                timeover() && return nothing
            end
            return roadmap
        end

        # swap tree
        V1, V2 = V2, V1
    end

    return nothing
end

"""used in RRT-Connect"""
function extend!(
    connect::Function,
    q_rand::State,  # sampled point
    V::Vector{State},  # vertices
    from_start::Bool,  # true -> tree rooted C_init, false -> tree rooted C_goal
    steering_depth::Int64,
)::Tuple{Symbol,State} where {State<:AbsState}

    # find nearest existing vertex
    q_near =
        (from_start ? argmin(q -> dist(q, q_rand), V) : argmin(q -> dist(q_rand, q), V))
    flg = :reached

    q_l = q_near  # safe sample
    q_h = q_rand  # probably unsafe sample -> eventually safe

    # steering, binary search
    if from_start ? !connect(q_near, q_h) : !(connect(q_h) && connect(q_h, q_near))
        flg = :trapped
        for _ = 1:steering_depth
            q = from_start ? get_mid_status(q_l, q_h) : get_mid_status(q_h, q_l)
            if from_start ? connect(q_near, q) : (connect(q) && connect(q, q_near))
                flg = :advanced
                q_l = q
            else
                q_h = q
            end
        end
        q_h = q_l
    end

    flg != :trapped && push!(V, q_h)
    return (flg, q_h)
end

"""generate id of search nodes"""
function get_Q_id(Q::Vector{Node{State}}, next::Int64)::String where {State<:AbsState}
    return @sprintf("%s_%d", join([v.id for v in Q], "-"), next)
end

"""obtain solution from search nodes by backtracking"""
function backtrack(
    S_fin::SuperNode{State},
    VISITED::Dict{String,SuperNode{State}},
)::Vector{Vector{Node{State}}} where {State<:AbsState}

    S = S_fin
    solution = Vector{Vector{Node{State}}}()
    while S.parent_id != nothing
        pushfirst!(solution, S.Q)
        S = VISITED[S.parent_id]
    end
    pushfirst!(solution, S.Q)
    return solution
end

end

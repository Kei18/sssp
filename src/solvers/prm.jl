"""Implementation of (simplified) PRM

ref:
- Kavraki, L. E., Svestka, P., Latombe, J. C., & Overmars, M. H. (1996).
  Probabilistic roadmaps for path planning in high-dimensional configuration spaces.
  IEEE transactions on Robotics and Automation
- Karaman, S., & Frazzoli, E. (2011).
  Sampling-based algorithms for optimal motion planning.
  The international journal of robotics research (IJRR)
"""

"""
    PRM_direct(
        config_init::Vector{State},
        config_goal::Vector{State},
        connect::Function,
        collide::Function,
        check_goal::Function;
        epsilon::Union{Float64,Nothing} = 0.2,
        TIME_LIMIT::Union{Nothing,Real} = nothing,
        VERBOSE::Int64 = 0,
    )::Tuple{
        Union{Nothing,Vector{Vector{Node{State}}}},  # solution
        Vector{Vector{Node{State}}},  # roadmaps
    } where {State<:AbsState}

solve the problem directly by PRM
"""
function PRM_direct(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function;
    epsilon::Union{Float64,Nothing} = 0.2,
    TIME_LIMIT::Union{Nothing,Real} = nothing,
    VERBOSE::Int64 = 0,
)::Tuple{
    Union{Nothing,Vector{Vector{Node{State}}}},  # solution
    Vector{Vector{Node{State}}},  # roadmaps
} where {State<:AbsState}

    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    N = length(config_init)

    # define utilities
    _sampler = MRMP.gen_uniform_sampling(config_init[1])
    sampler() = map(i -> _sampler(), 1:N)
    connect_C(C_from::Vector{State}, C_to::Vector{State}) = begin
        (isnothing(epsilon) || dist(C_from, C_to) <= epsilon) &&
        all(i -> connect(C_from[i], C_to[i], i), 1:N) &&
        begin
            for i = 1:N, j = i+1:N
                if collide(C_from[i], C_to[i], C_from[j], C_to[j], i, j)
                    return false
                end
            end
            return true
        end
    end

    get_roadmaps() = begin
        roadmaps = map(i -> Vector{Node{State}}(), 1:N)
        for (k, C) in enumerate(V)
            for (i, q) in enumerate(C)
                push!(roadmaps[i], Node{State}(q, k, E[k]))
            end
        end
        roadmaps
    end

    # vertices
    V = [config_init, config_goal]

    # edges
    E = [[], []]

    # distance, used with Dijkstra method
    D = [0.0, nothing]

    # special case
    if connect_C(config_init, config_goal)
        return (
            [
                map(q -> Node{State}(q, 1, []), config_init),
                map(q -> Node{State}(q, 2, []), config_goal),
            ],
            get_roadmaps(),
        )
    end

    iter = 0
    while !timeover()
        iter += 1

        VERBOSE > 1 &&
            iter % 100 == 0 &&
            @printf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)

        # sampling
        C = sampler()
        while !timeover() && any(i -> !connect(C[i], i), 1:N)
            C = sampler()
        end
        timeover() && break

        # insert new vertex
        push!(V, C)
        push!(D, nothing)
        L = length(V)
        # update neighbors
        neigh_from_C = collect(filter(k -> connect_C(C, V[k]), 1:L-1))
        neigh_to_C = collect(filter(k -> connect_C(V[k], C), 1:L-1))
        push!(E, neigh_from_C)
        foreach(k -> push!(E[k], L), neigh_to_C)

        isempty(neigh_from_C) && isempty(neigh_to_C) && continue

        # Dijkstra method
        # setup open list
        OPEN = PriorityQueue{Int64,Float64}()
        enqueue!(OPEN, 1, D[1])

        # parent
        P = fill(0, L)

        while !isempty(OPEN)
            # pop
            k = dequeue!(OPEN)

            # check goal
            if check_goal(V[k])
                # backtrack
                solution = Vector{Vector{Node{State}}}()
                l = k
                while l != 0
                    pushfirst!(solution, map(q -> Node{State}(q, l, []), V[l]))
                    l = P[l]
                end
                VERBOSE > 1 && @printf("\r\t%6.4f sec, iteration: %02d", elapsed(), iter)
                VERBOSE > 0 && @info @sprintf("\t%6.4f sec: found solution\n", elapsed())
                return (solution, get_roadmaps())
            end

            # expand
            for l in E[k]
                d = D[k] + dist(V[k], V[l])
                if isnothing(D[l]) || d < D[l]
                    # update connection
                    D[l] = d
                    P[l] = k
                    haskey(OPEN, l) && delete!(OPEN, l)
                    enqueue!(OPEN, l, d)
                end
            end
        end
    end

    VERBOSE > 0 && @info @sprintf("\n\t%6.4f sec: failed to find solution\n", elapsed())
    return (nothing, get_roadmaps())
end


"""
    PRM!(
        roadmap::Vector{Node{State}},
        connect::Function,
        num_vertices::Int64 = 100,
        rad::Union{Nothing,Float64} = nothing;
        TIME_LIMIT::Union{Nothing,Real} = nothing,
    )::Vector{Node{State}} where {State<:AbsState}

update known probabilistic roadmap
"""
function PRM!(
    roadmap::Vector{Node{State}},
    connect::Function,
    num_vertices::Int64 = 100,
    rad::Union{Nothing,Float64} = nothing;
    TIME_LIMIT::Union{Nothing,Real} = nothing,
)::Vector{Node{State}} where {State<:AbsState}

    t_s = now()
    elapsed() = elapsed_sec(t_s)
    timeover() = TIME_LIMIT != nothing && elapsed() > TIME_LIMIT

    sampler = MRMP.gen_uniform_sampling(roadmap[1].q)

    # sampling vertices
    while length(roadmap) < num_vertices && !timeover()
        q = sampler()
        # exclude sample not in free space
        if connect(q)
            push!(roadmap, Node{State}(q, length(roadmap) + 1, []))
        end
    end

    # identify neighbors
    for j = 1:num_vertices, k = 1:num_vertices
        timeover() && break
        j == k && continue
        q_from = roadmap[j].q
        q_to = roadmap[k].q
        if (isnothing(rad) || dist(q_from, q_to) <= rad) && connect(q_from, q_to)
            push!(roadmap[j].neighbors, k)
        end
    end

    return roadmap
end

"""
    PRM(
        q_init::State,
        q_goal::State,
        args...;
        kwargs...,
    )::Vector{Node{State}} where {State<:AbsState}

generate one PRM
"""
function PRM(
    q_init::State,
    q_goal::State,
    args...;
    kwargs...,
)::Vector{Node{State}} where {State<:AbsState}
    return PRM!(
        [Node{State}(q_init, 1, []), Node{State}(q_goal, 2, [])],
        args...;
        kwargs...,
    )
end

"""
    PRMs(
        config_init::Vector{State},
        config_goal::Vector{State},
        connect::Function,
        num_vertices::Int64 = 100;
        rad::Union{Nothing,Float64} = nothing,
        rads::Union{Vector{Nothing},Vector{Float64}} = fill(rad, length(config_init)),
        TIME_LIMIT::Union{Nothing,Real} = nothing,
    )::Vector{Vector{Node{State}}} where {State<:AbsState}

generate multiple PRMs
"""
function PRMs(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    num_vertices::Int64 = 100;
    rad::Union{Nothing,Float64} = nothing,
    rads::Union{Vector{Nothing},Vector{Float64}} = fill(rad, length(config_init)),
    TIME_LIMIT::Union{Nothing,Real} = nothing,
)::Vector{Vector{Node{State}}} where {State<:AbsState}

    t_s = now()
    elapsed() = elapsed_sec(t_s)

    N = length(config_init)
    return map(
        i -> PRM(
            config_init[i],
            config_goal[i],
            (args...) -> connect(args..., i),
            num_vertices,
            rads[i];
            TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
        ),
        1:N,
    )
end

"""
    PRMs!(
        roadmaps::Vector{Vector{Node{State}}},
        connect::Function,
        num_vertices::Int64 = 100;
        rad::Union{Nothing,Float64} = nothing,
        rads::Union{Vector{Nothing},Vector{Float64}} = fill(rad, length(roadmaps)),
        TIME_LIMIT::Union{Nothing,Real} = nothing,
    )::Vector{Vector{Node{State}}} where {State<:AbsState}

update PRMs
"""
function PRMs!(
    roadmaps::Vector{Vector{Node{State}}},
    connect::Function,
    num_vertices::Int64 = 100;
    rad::Union{Nothing,Float64} = nothing,
    rads::Union{Vector{Nothing},Vector{Float64}} = fill(rad, length(roadmaps)),
    TIME_LIMIT::Union{Nothing,Real} = nothing,
)::Vector{Vector{Node{State}}} where {State<:AbsState}

    t_s = now()
    elapsed() = elapsed_sec(t_s)

    N = length(roadmaps)
    return map(
        i -> PRM!(
            roadmaps[i],
            (args...) -> connect(args..., i),
            num_vertices,
            rads[i];
            TIME_LIMIT = (isnothing(TIME_LIMIT) ? nothing : TIME_LIMIT - elapsed()),
        ),
        1:N,
    )
end

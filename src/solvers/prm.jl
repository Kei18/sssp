function PRM!(
    roadmap::Vector{Node{State}},
    connect::Function,
    num_vertices::Int64 = 100,
    rad::Union{Nothing,Float64} = nothing;
    TIME_LIMIT::Union{Nothing,Real} = nothing,
)::Vector{Node{State}} where {State<:AbsState}
    """probabilistic roadmap"""

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
        if timeover()
            break
        end

        q_from = roadmap[j].q
        q_to = roadmap[k].q
        if (isnothing(rad) || dist(q_from, q_to) <= rad) && connect(q_from, q_to)
            push!(roadmap[j].neighbors, k)
        end
    end

    return roadmap
end

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

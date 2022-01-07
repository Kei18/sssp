function PRM!(
    roadmap::Vector{Node{State}},
    connect::Function,
    num_vertices::Int64 = 100,
    rad::Union{Nothing, Float64} = nothing,
    )::Vector{Node{State}} where {State<:AbsState}
    """probabilistic roadmap"""

    sampler = MRMP.gen_uniform_sampling(roadmap[1].q)

    # sampling vertices
    while length(roadmap) < num_vertices
        q = sampler()
        if connect(q)
            push!(roadmap, Node{State}(q, length(roadmap) + 1, []))
        end
    end

    # identify neighbors
    for j = 1:num_vertices, k = 1:num_vertices
        q_from = roadmap[j].q
        q_to = roadmap[k].q
        if (rad == nothing || dist(q_from, q_to) <= rad) && connect(q_from, q_to)
            push!(roadmap[j].neighbors, k)
        end
    end

    return roadmap
end

function PRM(q_init::State, q_goal::State, args...)::Vector{Node{State}} where {State<:AbsState}
    return PRM!([Node{State}(q_init, 1, []), Node{State}(q_goal, 2, [])], args...)
end

function PRMs(
    config_init::Vector{State},
    config_goal::Vector{State},
    connect::Function,
    num_vertices::Int64 = 100;
    rad::Union{Nothing, Float64} = nothing,
    rads::Union{Vector{Nothing}, Vector{Float64}} = fill(rad, length(config_init)),
)::Vector{Vector{Node{State}}} where {State<:AbsState}

    N = length(config_init)
    return map(
        i -> PRM(
            config_init[i],
            config_goal[i],
            (args...) -> connect(args..., i),
            num_vertices,
            rads[i],
        ),
        1:N
    )
end

function PRMs!(
    roadmaps::Vector{Vector{Node{State}}},
    connect::Function,
    num_vertices::Int64 = 100;
    rad::Union{Nothing, Float64} = nothing,
    rads::Union{Vector{Nothing}, Vector{Float64}} = fill(rad, length(roadmaps)),
    )::Vector{Vector{Node{State}}} where {State<:AbsState}

    N = length(roadmaps)
    return map(
        i -> PRM!(
            roadmaps[i],
            (args...) -> connect(args..., i),
            num_vertices,
            rads[i]
        ),
        1:N
    )
end

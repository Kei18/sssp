struct StatePoint2D <: StatePoint
    x::Float64
    y::Float64
end

to_string(s::StatePoint2D) = @sprintf("(%.4f, %.4f)", s.x, s.y)

function get_mid_status(p::StatePoint2D, q::StatePoint2D)::StatePoint2D
    return StatePoint2D((p.x + q.x) / 2, (p.y + q.y) / 2)
end

function dist(q1::StatePoint2D, q2::StatePoint2D)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y])
end

function dist(q_from::StatePoint2D, q_to::StatePoint2D, o::CircleObstacle2D)::Float64
    return dist([q_from.x, q_from.y], [q_to.x, q_to.y], [o.x, o.y])
end

function dist(q::StatePoint2D, o::CircleObstacle2D)::Float64
    return norm([q.x - o.x, q.y - o.y])
end

function dist(
    a_from::StatePoint2D,
    a_to::StatePoint2D,
    b_from::StatePoint2D,
    b_to::StatePoint2D,
)::Float64
    return dist(
        [a_from.x, a_from.y],
        [a_to.x, a_to.y],
        [b_from.x, b_from.y],
        [b_to.x, b_to.y],
    )
end

function gen_connect(
    q::StatePoint2D,  # to identify type
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle2D},
)::Function

    # check: q \in C_free
    f(q::StatePoint2D, i::Int64)::Bool = begin
        if any(x -> (x < rads[i] || 1 - rads[i] < x), [q.x, q.y])
            return false
        end

        if any([dist(q, o) < o.r + rads[i] for o in obstacles])
            return false
        end

        return true
    end

    f(q_from::StatePoint2D, q_to::StatePoint2D, i::Int64)::Bool = begin
        if any(x -> (x < rads[i] || 1 - rads[i] < x), [q_to.x, q_to.y])
            return false
        end

        # check: collisions with static obstacles
        if any([dist(q_from, q_to, o) < o.r + rads[i] for o in obstacles])
            return false
        end

        return true
    end

    return f
end

function gen_collide(q::StatePoint2D, rads::Vector{Float64})::Function
    @gen_collide(
        StatePoint2D,
        begin
            return dist(q_i_from, q_i_to, q_j_from, q_j_to) < rads[i] + rads[j]
        end,
        begin
            return dist(q_i, q_j) < rads[i] + rads[j]
        end,
    )
end

function gen_uniform_sampling(q::StatePoint2D)::Function
    () -> begin
        return StatePoint2D(rand(2)...)
    end
end

function plot_motion!(q_from::StatePoint2D, q_to::StatePoint2D, rad::Float64, params)
    plot!([q_from.x, q_to.x], [q_from.y, q_to.y]; params...)
end

function plot_start_goal!(q_init::StatePoint2D, q_goal::StatePoint2D, rad::Float64, params)
    plot!([q_init.x], [q_init.y]; markershape = :hex, params...)
    plot!([q_goal.x], [q_goal.y]; markershape = :star, params...)
end

function plot_agent!(q::StatePoint2D, rad::Float64, color::String)
    plot_circle!(q.x, q.y, rad, color, 3.0, fillalpha = 0.1)
end

function gen_random_instance_StatePoint2D(;
    N_min::Int64 = 2,
    N_max::Int64 = 8,
    N::Int64 = rand(N_min:N_max),
    num_obs_min::Int64 = 0,
    num_obs_max::Int64 = 10,
    num_obs::Int64 = rand(num_obs_min:num_obs_max),
    rad::Float64 = 0.025,
    rad_obs::Float64 = 0.05,
    rad_min::Float64 = rad,
    rad_max::Float64 = rad,
    rad_obs_min::Float64 = rad_obs,
    rad_obs_max::Float64 = rad_obs,
)

    # generate obstacles
    obstacles = map(
        k -> CircleObstacle2D(
            rand(2)...,
            rand() * (rad_obs_max - rad_obs_min) + rad_obs_min,
        ),
        1:num_obs,
    )

    # determine rads
    rads = map(e -> rand() * (rad_max - rad_min) + rad_min, 1:N)

    # generate
    q = StatePoint2D(0.0, 0.0)
    connect = gen_connect(q, rads, obstacles)
    collide = gen_collide(q, rads)
    sampler = gen_uniform_sampling(q)
    config_init = Vector{StatePoint2D}()
    config_goal = Vector{StatePoint2D}()
    for i = 1:N
        # add start
        isvalid_init =
            (q::StatePoint2D) -> (
                connect(q, i) &&
                all(e -> !collide(q, e[2], i, e[1]), enumerate(config_init))
            )
        q_init = sampler()
        while !isvalid_init(q_init)
            q_init = sampler()
        end
        push!(config_init, q_init)

        # add goal
        isvalid_goal =
            (q::StatePoint2D) -> (
                connect(q, i) &&
                all(e -> !collide(q, e[2], i, e[1]), enumerate(config_goal))
            )
        q_goal = sampler()
        while !isvalid_goal(q_goal)
            q_goal = sampler()
        end
        push!(config_goal, q_goal)
    end

    return (config_init, config_goal, obstacles, rads)
end

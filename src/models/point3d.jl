struct StatePoint3D <: StatePoint
    x::Float64
    y::Float64
    z::Float64
end

to_string(s::StatePoint3D) = @sprintf("(%.4f, %.4f, %.4f)", s.x, s.y, s.z)

function get_mid_status(p::StatePoint3D, q::StatePoint3D)::StatePoint3D
    return StatePoint3D((p.x + q.x) / 2, (p.y + q.y) / 2, (p.z + q.z) / 2)
end

function dist(q1::StatePoint3D, q2::StatePoint3D)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y, q1.z - q2.z])
end

function dist(q_from::StatePoint3D, q_to::StatePoint3D, o::CircleObstacle3D)::Float64
    return dist([q_from.x, q_from.y, q_from.z], [q_to.x, q_to.y, q_to.z], [o.x, o.y, o.z])
end

function dist(q::StatePoint3D, o::CircleObstacle3D)::Float64
    return norm([q.x - o.x, q.y - o.y, q.z - o.z])
end

function dist(
    a_from::StatePoint3D,
    a_to::StatePoint3D,
    b_from::StatePoint3D,
    b_to::StatePoint3D,
)::Float64
    return dist(
        [a_from.x, a_from.y, a_from.z],
        [a_to.x, a_to.y, a_to.z],
        [b_from.x, b_from.y, b_from.z],
        [b_to.x, b_to.y, b_to.z],
    )
end

function gen_connect(
    q::StatePoint3D,  # to identify type
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle3D},
    eps::Float64 = 0.2,
)::Function

    # check: q \in C_free
    f(q::StatePoint3D, i::Int64)::Bool = begin
        if any(x -> (x < rads[i] || 1 - rads[i] < x), [q.x, q.y, q.z])
            return false
        end

        if any([dist(q, o) < o.r + rads[i] for o in obstacles])
            return false
        end

        return true
    end

    f(q_from::StatePoint3D, q_to::StatePoint3D, i::Int64) = begin
        if any(x -> (x < rads[i] || 1 - rads[i] < x), [q_to.x, q_to.y, q_to.z])
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

function gen_collide(q::StatePoint3D, rads::Vector{Float64})::Function
    @gen_collide(
        StatePoint3D,
        begin
            return dist(q_i_from, q_i_to, q_j_from, q_j_to) < rads[i] + rads[j]
        end,
        begin
            return dist(q_i, q_j) < rads[i] + rads[j]
        end,
    )
end

function gen_uniform_sampling(q::StatePoint3D)::Function
    () -> begin
        return StatePoint3D(rand(3)...)
    end
end

function plot_motion!(q_from::StatePoint3D, q_to::StatePoint3D, rad::Float64, params)
    plot!([q_from.x, q_to.x], [q_from.y, q_to.y], [q_from.z, q_to.z]; params...)
end

function plot_start_goal!(q_init::StatePoint3D, q_goal::StatePoint3D, rad::Float64, params)
    plot!([q_init.x], [q_init.y], [q_init.z]; markershape = :hex, params...)
    plot!([q_goal.x], [q_goal.y], [q_goal.z]; markershape = :star, params...)
end

function plot_agent!(q::StatePoint3D, rad::Float64, color::String)
    plot_sphere!(q.x, q.y, q.z, rad, color)
end

function gen_random_instance_StatePoint3D(;
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
        k -> CircleObstacle3D(
            rand(3)...,
            rand() * (rad_obs_max - rad_obs_min) + rad_obs_min,
        ),
        1:num_obs,
    )

    # determine rads
    rads = map(e -> rand() * (rad_max - rad_min) + rad_min, 1:N)

    # generate
    q = StatePoint3D(0.0, 0.0, 0.0)
    connect = gen_connect(q, rads, obstacles)
    collide = gen_collide(q, rads)
    sampler = gen_uniform_sampling(q)
    config_init = Vector{StatePoint3D}()
    config_goal = Vector{StatePoint3D}()
    for i = 1:N
        # add start
        isvalid_init =
            (q::StatePoint3D) -> (
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
            (q::StatePoint3D) -> (
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

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

function dist(q_from::StatePoint3D, q_to::StatePoint3D, q_static::StatePoint3D)::Float64
    return dist(
        [q_from.x, q_from.y, q_from.z],
        [q_to.x, q_to.y, q_to.z],
        [q_static.x, q_static.y, q_static.z],
    )
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

function gen_random_instance_StatePoint3D(; params...)
    return gen_random_instance(StatePoint3D(0, 0, 0); params...)
end

struct StatePoint3D <: StatePoint
    x::Float64
    y::Float64
    z::Float64
end

function dist(q1::StatePoint3D, q2::StatePoint3D)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y, q1.z - q2.z])
end

function dist(q_from::StatePoint3D, q_to::StatePoint3D, o::CircleObstacle3D)::Float64
    return dist([q_from.x, q_from.y, q_from.z], [q_to.x, q_to.y, q_to.z], [o.x, o.y, o.z])
end

function dist(
    a_from::StatePoint3D, a_to::StatePoint3D,
    b_from::StatePoint3D, b_to::StatePoint3D)::Float64
    return dist([a_from.x, a_from.y, a_from.z], [a_to.x, a_to.y, a_to.z],
                [b_from.x, b_from.y, b_from.z], [b_to.x, b_to.y, b_from.z])
end

function gen_connect(
    q::StatePoint3D,  # to identify type
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle3D},
    eps::Float64=0.2
    )::Function

    return (q_from::StatePoint3D, q_to::StatePoint3D, i::Int64) -> begin
        # avoid fur points
        if dist(q_from, q_to) > eps; return false; end

        # check: q_to \in C_free
        if !all([rads[i] <= x <= 1 - rads[i] for x in [q_to.x, q_to.y, q_to.z]])
            return false
        end

        # check: collisions with static obstacles
        if any([ dist(q_from, q_to, o) < o.r + rads[i] for o in obstacles ])
            return false
        end

        return true
    end
    return f
end

function gen_collide(q::StatePoint3D, rads::Vector{Float64})::Function
    N = length(rads)
    return (Q_from::Vector{Node{StatePoint3D}}, Q_to::Vector{Node{StatePoint3D}}) -> begin
        for i = 1:N, j = i+1:N
            if dist(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q) < rads[i] + rads[j]
                return true
            end
        end
        return false
    end
end

function gen_random_walk(q::StatePoint3D, eps::Float64)::Function
    return (q::StatePoint3D) -> begin
        s = uniform_ball_sampling(3) * eps
        return StatePoint3D(q.x + s[1], q.y + s[2], q.z + s[3])
    end
end

function plot_motion!(q_from::StatePoint3D, q_to::StatePoint3D, rad::Float64, params)
    plot!([q_from.x, q_to.x], [q_from.y, q_to.y], [q_from.z, q_to.z]; params...)
end

function plot_start_goal!(q_init::StatePoint3D, q_goal::StatePoint3D, rad::Float64, params)
    plot!([q_init.x], [q_init.y], [q_init.z]; markershape=:hex, params...)
    plot!([q_goal.x], [q_goal.y], [q_init.z]; markershape=:star, params...)
end

function plot_agent!(q::StatePoint3D, rad::Float64, color::String)
    plot_sphere!(q.x, q.y, q.z, rad, color)
end

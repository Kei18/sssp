struct StatePoint2D <:StatePoint
    x::Float64
    y::Float64
end

function dist(q1::StatePoint2D, q2::StatePoint2D)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y])
end

function dist(q_from::StatePoint2D, q_to::StatePoint2D, o::CircleObstacle2D)::Float64
    return dist([q_from.x, q_from.y], [q_to.x, q_to.y], [o.x, o.y])
end

function dist(
    a_from::StatePoint2D, a_to::StatePoint2D,
    b_from::StatePoint2D, b_to::StatePoint2D)::Float64
    return dist([a_from.x, a_from.y], [a_to.x, a_to.y],
                [b_from.x, b_from.y], [b_to.x, b_to.y])
end

function gen_connect(
    q::StatePoint2D,  # to identify type
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle2D},
    eps::Float64=0.2
    )::Function

    return (q_from::StatePoint2D, q_to::StatePoint2D, i::Int64) -> begin
        # avoid far points
        if dist(q_from, q_to) > eps; return false; end

        # check: q_to \in C_free
        if !all([rads[i] <= x <= 1 - rads[i] for x in [q_to.x, q_to.y]])
            return false
        end

        # check: collisions with static obstacles
        if any([ dist(q_from, q_to, o) < o.r + rads[i] for o in obstacles ])
            return false
        end

        return true
    end
end

function gen_collide(q::StatePoint2D, rads::Vector{Float64})::Function
    N = length(rads)
    return (Q_from::Vector{Node{StatePoint2D}}, Q_to::Vector{Node{StatePoint2D}}) -> begin
        for i = 1:N, j = i+1:N
            if dist(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q) < rads[i] + rads[j]
                return true
            end
        end
        return false
    end
end

function gen_random_walk(q::StatePoint2D, eps::Float64)::Function
    return (q::StatePoint2D) -> begin
        s = uniform_ball_sampling(2) * eps
        return StatePoint2D(q.x + s[1], q.y + s[2])
    end
end

function plot_motion!(q_from::StatePoint2D, q_to::StatePoint2D, rad::Float64, params)
    plot!([q_from.x, q_to.x], [q_from.y, q_to.y]; params...)
end

function plot_start_goal!(q_init::StatePoint2D, q_goal::StatePoint2D, rad::Float64, params)
    plot!([q_init.x], [q_init.y]; markershape=:hex, params...)
    plot!([q_goal.x], [q_goal.y]; markershape=:star, params...)
end

function plot_agent!(q::StatePoint2D, rad::Float64, color::String)
    plot_circle!(q.x, q.y, rad, color, 3.0, fillalpha=0.1)
end

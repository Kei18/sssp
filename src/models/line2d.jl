struct StateLine2D <: AbsState
    x::Float64
    y::Float64
    theta::Float64
end

function dist(q1::StateLine2D, q2::StateLine2D)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y])
end

function gen_connect(
    q::StateLine2D,  # to identify type
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle2D},
    eps::Float64=0.2;
    step=10
    )::Function

    return (q_from::StateLine2D, q_to::StateLine2D, i::Int64) -> begin
        # avoid far points
        if norm([q_from.x - q_to.x, q_from.y - q_to.y,
                 (q_from.theta - q_to.theta) / π ]) > eps
            return false
        end

        for e=(0:step)/step
            a = (1-e) * [q_from.x, q_from.y] + e * [q_to.x, q_to.y]
            t = (1-e) * q_from.theta + e * q_to.theta
            b = a + rads[i] * [ cos(t), sin(t) ]

            # outside
            if !all([ 0 <= x <= 1 for x in vcat(a, b) ]); return false; end

            # obstacles
            if any([ dist(a, b, [o.x, o.y]) < o.r for o in obstacles ]); return false; end
        end

        return true
    end
end


function gen_collide(q::StateLine2D, rads::Vector{Float64}; step=10)::Function
    N = length(rads)
    return (Q_from::Vector{Node{StateLine2D}}, Q_to::Vector{Node{StateLine2D}}) -> begin
        for i = 1:N, j = i+1:N
            q_i_from = Q_from[i].q
            q_i_to = Q_to[i].q
            q_j_from = Q_from[j].q
            q_j_to = Q_to[j].q
            for e_i=(0:step)/step, e_j=(0:step)/step
                # from
                a_i = (1-e_i) * [q_i_from.x, q_i_from.y] + e_i * [q_i_to.x, q_i_to.y]
                a_j = (1-e_j) * [q_j_from.x, q_j_from.y] + e_j * [q_j_to.x, q_j_to.y]
                # angle
                t_i = (1-e_i) * q_i_from.theta + e_i * q_i_to.theta
                t_j = (1-e_j) * q_j_from.theta + e_j * q_j_to.theta
                # to
                b_i = rads[i] * [ cos(t_i), sin(t_i) ] + a_i
                b_j = rads[j] * [ cos(t_j), sin(t_j) ] + a_j
                # check collision
                if segments_intersect(a_i, b_i, a_j, b_j); return true; end
            end
        end
        return false
    end
end

function gen_random_walk(q::StateLine2D, eps::Float64)::Function
    return (q::StateLine2D) -> begin
        s = uniform_ball_sampling(3) * eps
        return StateLine2D(s[1] + q.x, s[2] + q.y, s[3] * π + q.theta)
    end
end

function plot_motion!(q_from::StateLine2D, q_to::StateLine2D, rad::Float64, params)
    plot!([q_from.x, q_to.x], [q_from.y, q_to.y]; params...)
end

function plot_start_goal!(q_init::StateLine2D, q_goal::StateLine2D, rad::Float64, params)
    plot!([q_init.x], [q_init.y]; markershape=:hex, params...)
    plot!([q_goal.x], [q_goal.y]; markershape=:star, params...)
end

function plot_agent!(q::StateLine2D, rad::Float64, color::String)
    plot!([q.x, rad * cos(q.theta) + q.x], [q.y, rad * sin(q.theta) + q.y],
          color=color, lw=5, label=nothing)
end

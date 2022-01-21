struct StateLine2D <: AbsState
    x::Float64
    y::Float64
    theta::Float64
end

const STEP_DIST_LINE2D = 0.01

to_string(s::StateLine2D) = @sprintf("(%.4f, %.4f, theta: %.4f)", s.x, s.y, s.theta)

function get_mid_status(p::StateLine2D, q::StateLine2D)::StateLine2D
    return StateLine2D(
        (p.x + q.x) / 2,
        (p.y + q.y) / 2,
        diff_angles(q.theta, p.theta) / 2 + p.theta,
    )
end

function dist(q1::StateLine2D, q2::StateLine2D)::Float64
    # normalize
    return norm([q1.x - q2.x, q1.y - q2.y, diff_angles(q1.theta, q2.theta) / π])
end

function gen_connect(
    q::StateLine2D,  # to identify type
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle2D};
    step_dist::Float64 = STEP_DIST_LINE2D,
    max_dist::Union{Nothing,Float64} = nothing,
)::Function

    # check: q \in C_free
    f(q::StateLine2D, i::Int64)::Bool = begin
        a = [q.x, q.y]
        b = [cos(q.theta), sin(q.theta)] * rads[i] + a

        if any(x -> (x < rads[i] || 1 - rads[i] < x), vcat(a, b))
            return false
        end

        if any(o -> dist(a, b, [o.x, o.y]) < o.r, obstacles)
            return false
        end

        return true
    end

    f(q_from::StateLine2D, q_to::StateLine2D, i::Int64)::Bool = begin
        D = dist(q_from, q_to)
        !isnothing(max_dist) && D > max_dist && return false

        dt = diff_angles(q_to.theta, q_from.theta)
        for e in vcat(collect(0:step_dist:D) / D, 1.0)
            # intermediate: root
            a = (1 - e) * [q_from.x, q_from.y] + e * [q_to.x, q_to.y]
            # intermediate: theta
            t = q_from.theta + e * dt
            # intermediate: tip
            b = a + rads[i] * [cos(t), sin(t)]

            # outside
            any(x -> (x < 0 || 1 < x), vcat(a, b)) && return false

            # obstacles
            any(o -> dist(a, b, [o.x, o.y]) < o.r, obstacles) && return false
        end

        return true
    end

    return f
end


function gen_collide(
    q::StateLine2D,
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST_LINE2D,
    safety_dist::Float64 = 0.01,
)::Function

    N = length(rads)

    f(
        q_i_from::StateLine2D,
        q_i_to::StateLine2D,
        q_j_from::StateLine2D,
        q_j_to::StateLine2D,
        i::Int64,
        j::Int64,
    ) = begin
        # check each pair of step
        D_i = dist(q_i_from, q_i_to)
        D_j = dist(q_j_from, q_j_to)

        dt_i = diff_angles(q_i_to.theta, q_i_from.theta)
        dt_j = diff_angles(q_j_to.theta, q_j_from.theta)

        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # root, angle, tip
            a_i = (1 - e_i) * [q_i_from.x, q_i_from.y] + e_i * [q_i_to.x, q_i_to.y]
            t_i = q_i_from.theta + e_i * dt_i
            b_i = rads[i] * [cos(t_i), sin(t_i)] + a_i

            for e_j in vcat(collect(0:step_dist:D_j) / D_j, 1.0)
                # root, angle, tip
                a_j = (1 - e_j) * [q_j_from.x, q_j_from.y] + e_j * [q_j_to.x, q_j_to.y]
                t_j = q_j_from.theta + e_j * dt_j
                b_j = rads[j] * [cos(t_j), sin(t_j)] + a_j

                # check collision
                if dist(a_i, b_i, a_j, b_j) < safety_dist
                    return true
                end
            end
        end
        return false
    end

    f(q_i::StateLine2D, q_j::StateLine2D, i::Int64, j::Int64) = begin
        a_i = [q_i.x, q_i.y]
        b_i = [cos(q_i.theta), sin(q_i.theta)] * rads[i] + a_i
        a_j = [q_j.x, q_j.y]
        b_j = [cos(q_j.theta), sin(q_j.theta)] * rads[j] + a_j
        return dist(a_i, b_i, a_j, b_j) < safety_dist
    end

    f(Q_from::Vector{Node{StateLine2D}}, Q_to::Vector{Node{StateLine2D}}) = begin
        for i = 1:N, j = i+1:N
            if f(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q, i, j)
                return true
            end
        end
        return false
    end

    f(Q::Vector{Node{StateLine2D}}, q_i_to::StateLine2D, i::Int64) = begin
        q_i_from = Q[i].q
        D_i = dist(q_i_from, q_i_to)

        dt_i = diff_angles(q_i_to.theta, q_i_from.theta)

        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # root, angle, tip
            a_i = (1 - e_i) * [q_i_from.x, q_i_from.y] + e_i * [q_i_to.x, q_i_to.y]
            t_i = q_i_from.theta + e_i * dt_i
            b_i = rads[i] * [cos(t_i), sin(t_i)] + a_i

            for j in filter(k -> k != i, 1:N)
                q_j = Q[j].q
                a_j = [q_j.x, q_j.y]
                b_j = rads[j] * [cos(q_j.theta), sin(q_j.theta)] + a_j

                # check collision
                if dist(a_i, b_i, a_j, b_j) < safety_dist
                    return true
                end
            end
        end

        return false
    end

    return f
end

function gen_uniform_sampling(q::StateLine2D)::Function
    () -> begin
        return StateLine2D(rand(2)..., rand() * 2π)
    end
end

function plot_motion!(q_from::StateLine2D, q_to::StateLine2D, rad::Float64, params)
    plot!([q_from.x, q_to.x], [q_from.y, q_to.y]; params...)
end

function plot_start_goal!(q_init::StateLine2D, q_goal::StateLine2D, rad::Float64, params)
    plot!([q_init.x], [q_init.y]; markershape = :hex, params...)
    plot!([q_goal.x], [q_goal.y]; markershape = :star, params...)
end

function plot_agent!(q::StateLine2D, rad::Float64, color::String)
    plot!(
        [q.x, rad * cos(q.theta) + q.x],
        [q.y, rad * sin(q.theta) + q.y],
        color = color,
        lw = 5,
        label = nothing,
    )
end

function gen_random_instance_StateLine2D(; params...)
    return gen_random_instance(StateLine2D(0, 0, 0); params...)
end

struct StateArm2 <: AbsState
    x::Float64
    y::Float64
    theta1::Float64
    theta2::Float64
end

function dist(q1::StateArm2, q2::StateArm2)
    t1 = abs(mod(q1.theta1, 2π) - mod(q2.theta1, 2π))
    t1 = minimum([t1, 2π - t1])
    t2 = abs(mod(q1.theta2, 2π) - mod(q2.theta2, 2π))
    t2 = minimum([t2, 2π - t2])
    return norm([t1 / π, t2 / π])
end

function get_arm_intermediate_point(q::StateArm2, rad::Float64)
    x = q.x + cos(q.theta1) * rad
    y = q.y + sin(q.theta1) * rad
    return [x, y]
end

function get_arm_tip_point(q::StateArm2, rad::Float64)
    pos = get_arm_intermediate_point(q, rad)
    t = q.theta2 + q.theta1
    x = pos[1] + cos(t) * rad
    y = pos[2] + sin(t) * rad
    return [x, y]
end

function gen_connect(
    q::StateArm2,  # to identify type
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle2D},
    eps::Float64=0.2;
    step::Int64=10
    )::Function

    return (q_from::StateArm2, q_to::StateArm2, i::Int64) -> begin
        # avoid far points
        if dist(q_from, q_to) > eps; return false; end

        # root
        p0 = [q_from.x, q_from.y]

        # angular change
        d1 = atan(sin(q_to.theta1 - q_from.theta1), cos(q_to.theta1 - q_from.theta1))
        d2 = atan(sin(q_to.theta2 - q_from.theta2), cos(q_to.theta2 - q_from.theta2))

        # check self-collision
        if π/2 < mod(q_from.theta2, 2π) < π && π < mod(q_to.theta2, 2π) < 3π/2
            return false
        end
        if π/2 < mod(q_to.theta2, 2π) < π && π < mod(q_from.theta2, 2π) < 3π/2
            return false
        end

        # check motion
        for e=(0:step)/step
            t1 = d1*e + q_from.theta1
            t2 = d2*e + q_from.theta2
            q = StateArm2(q_from.x, q_from.y, t1, t2)
            p1 = get_arm_intermediate_point(q, rads[i])
            p2 = get_arm_tip_point(q, rads[i])

            # check: points are in C_free
            if !all(0 <= z <= 1 for z in vcat(p1, p2)); return false; end

            # check: collisions with obstacles
            if any([ dist(p0, p1, [o.x, o.y]) < o.r for o in obstacles ]); return false; end
            if any([ dist(p1, p2, [o.x, o.y]) < o.r for o in obstacles ]); return false; end
        end

        return true
    end
end

function gen_collide(q::StateArm2, rads::Vector{Float64}; step::Int64=10)::Function
    N = length(rads)

    return (Q_from::Vector{Node{StateArm2}}, Q_to::Vector{Node{StateArm2}}) -> begin
        for i = 1:N, j = i+1:N
            q_i_from = Q_from[i].q
            q_i_to   = Q_to[i].q
            q_j_from = Q_from[j].q
            q_j_to   = Q_to[j].q

            # root
            p_i_0 = [q_i_from.x, q_i_from.y]
            p_j_0 = [q_j_from.x, q_j_from.y]

            # conservative check
            p_i_1 = get_arm_intermediate_point(q_i_from, rads[i])
            p_i_2 = get_arm_intermediate_point(q_i_to, rads[i])
            p_i_3 = get_arm_tip_point(q_i_from, rads[i])
            p_i_4 = get_arm_tip_point(q_i_to, rads[i])

            p_j_1 = get_arm_intermediate_point(q_j_from, rads[j])
            p_j_2 = get_arm_intermediate_point(q_j_to, rads[j])
            p_j_3 = get_arm_tip_point(q_j_from, rads[j])
            p_j_4 = get_arm_tip_point(q_j_to, rads[j])

            if segments_intersect(p_i_3, p_i_4, p_j_0, p_j_1); return true; end
            if segments_intersect(p_i_3, p_i_4, p_j_0, p_j_2); return true; end
            if segments_intersect(p_i_3, p_i_4, p_j_1, p_j_3); return true; end
            if segments_intersect(p_i_3, p_i_4, p_j_2, p_j_4); return true; end

            if segments_intersect(p_j_3, p_j_4, p_i_0, p_i_1); return true; end
            if segments_intersect(p_j_3, p_j_4, p_i_0, p_i_2); return true; end
            if segments_intersect(p_j_3, p_j_4, p_i_1, p_i_3); return true; end
            if segments_intersect(p_j_3, p_j_4, p_i_2, p_i_4); return true; end

            # exact check (but approximated)
            for e_i=(0:step)/step, e_j=(0:step)/step
                t_i_1 = (mod(q_i_to.theta1, 2π) - mod(q_i_from.theta1, 2π))*e_i
                t_i_1 += q_i_from.theta1
                t_i_2 = (mod(q_i_to.theta2, 2π) - mod(q_i_from.theta2, 2π))*e_i
                t_i_2 += q_i_from.theta2
                t_j_1 = (mod(q_j_to.theta1, 2π) - mod(q_j_from.theta1, 2π))*e_j
                t_j_1 += q_j_from.theta1
                t_j_2 = (mod(q_j_to.theta2, 2π) - mod(q_j_from.theta2, 2π))*e_j
                t_j_2 += q_j_from.theta2
                q_i = StateArm2(p_i_0..., t_i_1, t_i_2)
                q_j = StateArm2(p_j_0..., t_j_1, t_j_2)
                p_i_1 = get_arm_intermediate_point(q_i, rads[i])
                p_i_2 = get_arm_tip_point(q_i, rads[i])
                p_j_1 = get_arm_intermediate_point(q_j, rads[j])
                p_j_2 = get_arm_tip_point(q_j, rads[j])

                # collision check
                if segments_intersect(p_i_0, p_i_1, p_j_0, p_j_1); return true; end
                if segments_intersect(p_i_0, p_i_1, p_j_1, p_j_2); return true; end
                if segments_intersect(p_i_1, p_i_2, p_j_0, p_j_1); return true; end
                if segments_intersect(p_i_1, p_i_2, p_j_1, p_j_2); return true; end
            end
        end
        return false
    end
end

function gen_random_walk(q::StateArm2, eps::Float64)::Function
    return (q::StateArm2) -> begin
        s = uniform_ball_sampling(2) * eps
        return StateArm2(q.x, q.y, s[1]*π + q.theta1, s[2]*π + q.theta2)
    end
end

function plot_motion!(q_from::StateArm2, q_to::StateArm2, rad::Float64, params)
    p_from = get_arm_tip_point(q_from, rad)
    p_to = get_arm_tip_point(q_to, rad)
    plot!([p_from[1], p_to[1]], [p_from[2], p_to[2]]; params...)
end

function plot_start_goal!(q_init::StateArm2, q_goal::StateArm2, rad::Float64, params)
    pos_init = get_arm_tip_point(q_init, rad)
    pos_goal = get_arm_tip_point(q_goal, rad)
    plot!([pos_init[1]], [pos_init[2]]; markershape=:hex, params...)
    plot!([pos_goal[1]], [pos_goal[2]]; markershape=:star, params...)
end

function plot_agent!(q::StateArm2, rad::Float64, color::String)
    pos1 = get_arm_intermediate_point(q, rad)
    pos2 = get_arm_tip_point(q, rad)
    plot!([q.x, pos1[1], pos2[1]], [q.y, pos1[2], pos2[2]], color=color,
          lw=5, markershape=:circle, markersize=3, label=nothing)
    scatter!([q.x], [q.y], markershpa=10, markershape=:hex, color=color, label=nothing)
end

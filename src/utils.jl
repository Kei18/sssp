export gen_connect_point, gen_collide_point, gen_collide_line, gen_check_goal
export gen_random_walk, gen_h_func, gen_g_func, gen_get_sample_nums
export gen_connect_line, gen_collide_line
export gen_connect_arm2, gen_collide_arm2

using LinearAlgebra: norm, dot, normalize

function direction(p_i::Vector{Float64}, p_j::Vector{Float64}, p_k::Vector{Float64})::Float64
    a = p_k - p_i
    b = p_j - p_i
    return a[1] * b[2] - b[1] * a[2]
end

function segments_intersect(
    p1::Vector{Float64}, p2::Vector{Float64}, p3::Vector{Float64}, p4::Vector{Float64})::Bool
    return (
        (direction(p3, p4, p1)*direction(p3, p4, p2) < 0) &&
        (direction(p1, p2, p3)*direction(p1, p2, p4) < 0)
    )
end

# from line to point
function dist(a::Vector{Float64}, b::Vector{Float64}, c::Vector{Float64})::Float64
    df_0 = dot(a-b, b-c)
    df_1 = dot(a-b, a-c)

    e = 1
    if df_0*df_1 < 0
        e = - dot(a-b, b-c) / dot(a-b, a-b)
    elseif df_0 > 0
        e = 0
    end

    return norm(e*a + (1-e)*b - c)
end

# from line to line
function dist(
    p1_1::Vector{Float64}, p1_2::Vector{Float64},
    p2_1::Vector{Float64}, p2_2::Vector{Float64})::Float64

    # c.f., http://marupeke296.com/COL_3D_No19_LinesDistAndPos.html
    v1 = p1_2 - p1_1
    v2 = p2_2 - p2_1
    D1 = dot(p2_1 - p1_1, v1)
    D2 = dot(p2_1 - p1_1, v2)
    Dv = dot(v1, v2)
    V1 = dot(v1, v1)
    V2 = dot(v2, v2)
    D = V1*V2 - Dv*Dv
    if D > 0
        t1 = (D1*V2 - D2*Dv) / D
        t2 = (D1*Dv - D2*V1) / D
        if 0 <= t1 <= 1 && 0 <= t2 <= 1
            Q1 = p1_1 + t1*v1
            Q2 = p2_1 + t2*v2
            return norm(Q1 - Q2)
        end
    end

    return minimum([dist(p1_1, p1_2, p2_1)
                    dist(p1_1, p1_2, p2_2)
                    dist(p2_1, p2_2, p1_1)
                    dist(p2_1, p2_2, p1_2)])
end

function dist(q1::StatePoint2D, q2::StatePoint2D)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y])
end

function dist(q1::StatePoint3D, q2::StatePoint3D)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y, q1.z - q2.z])
end

function dist(q1::StateLine2D, q2::StateLine2D)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y])
end

function dist(q1::StateArm2, q2::StateArm2)
    t1 = abs(mod(q1.theta1, 2π) - mod(q2.theta1, 2π))
    t1 = minimum([t1, 2π - t1])
    t2 = abs(mod(q1.theta2, 2π) - mod(q2.theta2, 2π))
    t2 = minimum([t2, 2π - t2])
    return norm([t1 / π, t2 / π])
end

function dist(q_from::StatePoint2D, q_to::StatePoint2D, o::CircleObstacle2D)::Float64
    return dist([q_from.x, q_from.y], [q_to.x, q_to.y], [o.x, o.y])
end

function dist(q_from::StatePoint3D, q_to::StatePoint3D, o::CircleObstacle3D)::Float64
    return dist([q_from.x, q_from.y, q_from.z], [q_to.x, q_to.y, q_to.z], [o.x, o.y, o.z])
end

function dist(
    a_from::StatePoint2D, a_to::StatePoint2D,
    b_from::StatePoint2D, b_to::StatePoint2D)::Float64
    return dist([a_from.x, a_from.y], [a_to.x, a_to.y], [b_from.x, b_from.y], [b_to.x, b_to.y])
end

function dist(
    a_from::StatePoint3D, a_to::StatePoint3D,
    b_from::StatePoint3D, b_to::StatePoint3D)::Float64
    return dist([a_from.x, a_from.y, a_from.z], [a_to.x, a_to.y, a_to.z],
                [b_from.x, b_from.y, b_from.z], [b_to.x, b_to.y, b_from.z])
end

function gen_collide_point(rads::Vector{Float64})::Function
    N = length(rads)
    return (Q_from::Union{Vector{Node{StatePoint2D}}, Vector{Node{StatePoint3D}}},
            Q_to::Union{Vector{Node{StatePoint2D}}, Vector{Node{StatePoint3D}}}) -> begin
        for i = 1:N, j = i+1:N
            if dist(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q) < rads[i] + rads[j]
                return true
            end
        end
        return false
    end
end

function gen_collide_line(rads::Vector{Float64}; step=10)::Function
    N = length(rads)
    return (Q_from::Vector{Node{StateLine2D}}, Q_to::Vector{Node{StateLine2D}}) -> begin
        for i = 1:N, j = i+1:N
            q_i_from = Q_from[i].q
            q_i_to = Q_to[i].q
            q_j_from = Q_from[j].q
            q_j_to = Q_to[j].q
            for e_i=(0:step)/step, e_j=(0:step)/step
                a_i = (1-e_i) * [q_i_from.x, q_i_from.y] + e_i * [q_i_to.x, q_i_to.y]
                a_j = (1-e_j) * [q_j_from.x, q_j_from.y] + e_j * [q_j_to.x, q_j_to.y]
                t_i = (1-e_i) * q_i_from.theta + e_i * q_i_to.theta
                t_j = (1-e_j) * q_j_from.theta + e_j * q_j_to.theta
                b_i = rads[i] * [ cos(t_i), sin(t_i) ] + a_i
                b_j = rads[j] * [ cos(t_j), sin(t_j) ] + a_j
                if segments_intersect(a_i, b_i, a_j, b_j); return true; end
            end
        end
        return false
    end
end

function gen_connect_point(
    rads::Vector{Float64}, obstacles::Vector{Obs} where Obs<:CircleObstacle, eps::Float64=0.2
    )::Function

    return (q_from::StatePoint, q_to::StatePoint, i::Int64) -> begin
        # avoid fur points
        if dist(q_from, q_to) > eps; return false; end

        # check: q_to \in C_free
        if isa(q_to, StatePoint2D)
            if !all([rads[i] <= x <= 1 - rads[i] for x in [q_to.x, q_to.y]]); return false; end
        else
            if !all([rads[i] <= x <= 1 - rads[i] for x in [q_to.x, q_to.y, q_to.z]]); return false; end
        end

        # check: collisions with static obstqacles
        if any([ dist(q_from, q_to, o) < o.r + rads[i] for o in obstacles ])
            return false
        end

        return true
    end
    return f
end

function gen_connect_line(
    rads::Vector{Float64}, obstacles::Vector{CircleObstacle2D}, eps::Float64=0.2; step=10
    )::Function

    return (q_from::StateLine2D, q_to::StateLine2D, i::Int64) -> begin
        if norm([q_from.x - q_to.x, q_from.y - q_to.y, (q_from.theta - q_to.theta) / π ]) > eps
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

function gen_connect_arm2(
    rads::Vector{Float64}, obstacles::Vector{CircleObstacle2D}, eps::Float64=0.2; step::Int64=10)::Function

    return (q_from::StateArm2, q_to::StateArm2, i::Int64) -> begin
        if dist(q_from, q_to) > eps; return false; end

        p0 = [q_from.x, q_from.y]
        d1 = atan(sin(q_to.theta1 - q_from.theta1), cos(q_to.theta1 - q_from.theta1))
        d2 = atan(sin(q_to.theta2 - q_from.theta2), cos(q_to.theta2 - q_from.theta2))

        # check self-collision
        if π/2 < mod(q_from.theta2, 2π) < π && π < mod(q_to.theta2, 2π) < 3π/2; return false; end
        if π/2 < mod(q_to.theta2, 2π) < π && π < mod(q_from.theta2, 2π) < 3π/2; return false; end

        for e=(0:step)/step
            t1 = d1*e + q_from.theta1
            t2 = d2*e + q_from.theta2
            q = StateArm2(q_from.x, q_from.y, t1, t2)
            p1 = get_arm_intermediate_point(q, rads[i])
            p2 = get_arm_tip_point(q, rads[i])
            if !all(0 <= z <= 1 for z in vcat(p1, p2)); return false; end

            # obstacles
            if any([ dist(p0, p1, [o.x, o.y]) < o.r for o in obstacles ]); return false; end
            if any([ dist(p1, p2, [o.x, o.y]) < o.r for o in obstacles ]); return false; end
        end

        return true
    end
end

function gen_collide_arm2(rads::Vector{Float64}; step::Int64=10)::Function
    N = length(rads)
    return (Q_from::Vector{Node{StateArm2}}, Q_to::Vector{Node{StateArm2}}) -> begin
        for i = 1:N, j = i+1:N
            q_i_from = Q_from[i].q
            q_i_to   = Q_to[i].q
            q_j_from = Q_from[j].q
            q_j_to   = Q_to[j].q
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
                t_i_1 = (mod(q_i_to.theta1, 2π) - mod(q_i_from.theta1, 2π))*e_i + q_i_from.theta1
                t_i_2 = (mod(q_i_to.theta2, 2π) - mod(q_i_from.theta2, 2π))*e_i + q_i_from.theta2
                t_j_1 = (mod(q_j_to.theta1, 2π) - mod(q_j_from.theta1, 2π))*e_j + q_j_from.theta1
                t_j_2 = (mod(q_j_to.theta2, 2π) - mod(q_j_from.theta2, 2π))*e_j + q_j_from.theta2
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

function gen_random_walk(eps::Float64)::Function
    origin(d::Int64) = begin
        f = () -> rand(d) * 2 - fill(1, d)
        s = f()
        while norm(s) > 1; s = f(); end
        return s * eps
    end

    return (q::AbsState) -> begin
        if isa(q, StatePoint2D)
            s = origin(2)
            return StatePoint2D(q.x + s[1], q.y + s[2])
        elseif isa(q, StatePoint3D)
            s = origin(3)
            return StatePoint3D(q.x + s[1], q.y + s[2], q.z + s[3])
        elseif isa(q, StateLine2D)
            s = origin(3)
            return StateLine2D(s[1] + q.x, s[2] + q.y, s[3] * π + q.theta)
        elseif isa(q, StateArm2)
            s = origin(2)
            return StateArm2(q.x, q.y, s[1]*π + q.theta1, s[2]*π + q.theta2)
        end
        return copy(q)
    end
end

function gen_check_goal(
    config_goal::Vector{State};
    goal_rad::Float64=0.0,
    goal_rads::Vector{Float64}=fill(goal_rad, length(config_goal))
    )::Function where State<:AbsState

    return (Q::Vector{Node{State}}) -> begin
        return all([dist(v.q, q) <= goal_rads[i]
                    for (i, (v, q)) in enumerate(zip(Q, config_goal))])
    end
end

function gen_h_func(config_goal::Vector{State})::Function where State<:AbsState
    return (Q::Vector{Node{State}}) -> begin
        return sum([dist(v.q, config_goal[i]) for (i, v) in enumerate(Q)])
    end
end

function gen_g_func(config_init::Vector{State}; greedy::Bool=false)::Function where State<:AbsState
    return (Q_from::Vector{Node{State}}, Q_to::Vector{Node{State}}) -> begin
        return greedy ? 0 : sum([dist(u.q, v.q) for (u, v) in zip(Q_from, Q_to)])
    end
end

function gen_get_sample_nums(k_init::Int64)::Function
    return (k::Int64) -> k + k_init - 1
end

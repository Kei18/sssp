export gen_connect_point, gen_collide_point, gen_check_goal
export gen_random_walk, gen_h_func, gen_g_func, gen_get_sample_nums
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

    # http://marupeke296.com/COL_3D_No19_LinesDistAndPos.html
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

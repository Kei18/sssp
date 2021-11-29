export dist

using LinearAlgebra: norm, dot, cross

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

function dist(q1::StatePoint2D, q2::StatePoint2D)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y])
end

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

function dist(
    p1_1::Vector{Float64}, p1_2::Vector{Float64},
    p2_1::Vector{Float64}, p2_2::Vector{Float64})::Float64

    if segments_intersect(p1_1, p1_2, p2_1, p2_2); return 0; end
    return minimum([dist(p1_1, p1_2, p2_1)
                    dist(p1_1, p1_2, p2_2)
                    dist(p2_1, p2_2, p1_1)
                    dist(p2_1, p2_2, p1_2)])
end

function dist(
    a_from::StatePoint2D, a_to::StatePoint2D,
    b_from::StatePoint2D, b_to::StatePoint2D)::Float64
    return dist([a_from.x, a_from.y], [a_to.x, a_to.y], [b_from.x, b_from.y], [b_to.x, b_to.y])
end

function gen_collide_point2d(rads::Vector{Float64})::Function
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

function gen_connect_point2d(
    rads::Vector{Float64}, obstacles::Vector{CircleObstacle2D}, eps::Float64=0.2)::Function
    return (q_from::StatePoint2D, q_to::StatePoint2D, i::Int64) -> begin
        # avoid fur points
        if dist(q_from, q_to) > eps; return false; end

        # check: q_to \in C_free
        if !all([rads[i] <= x <= 1 - rads[i] for x in [q_to.x, q_to.y]]); return false; end

        # check: collisions with static obstqacles
        if any([ dist([q_from.x, q_from.y], [q_to.x, q_to.y], [o.x, o.y]) < o.r + rads[i] for o in obstacles ])
            return false
        end

        return true
    end
    return f
end

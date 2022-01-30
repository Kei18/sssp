"""parent of StatePoint2D, StatePoint3D"""

abstract type StatePoint <: AbsState end

function gen_collide(q::State, rads::Vector{Float64})::Function where {State<:StatePoint}
    """collision function for point robots"""
    N = length(rads)

    f(q_i_from::State, q_i_to::State, q_j_from::State, q_j_to::State, i::Int64, j::Int64) =
        begin
            return dist(q_i_from, q_i_to, q_j_from, q_j_to) < rads[i] + rads[j]
        end

    f(q_i::State, q_j::State, i::Int64, j::Int64) = begin
        return dist(q_i, q_j) < rads[i] + rads[j]
    end

    f(Q_from::Vector{Node{State}}, Q_to::Vector{Node{State}}) = begin
        for i = 1:N, j = i+1:N
            if f(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q, i, j)
                return true
            end
        end
        return false
    end

    f(Q::Vector{Node{State}}, q_to::State, i::Int64) = begin
        q_from = Q[i].q
        for j = 1:N
            j != i && dist(q_from, q_to, Q[j].q) < rads[i] + rads[j] && return true
        end
        return false
    end

    return f
end

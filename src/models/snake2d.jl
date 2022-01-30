"""model definition of snake2d"""

const STEP_DIST_SNAKE2D = 0.05

struct StateSnake2D <: AbsState
    x::Float64
    y::Float64
    theta1::Float64
    theta2::Float64
    theta3::Float64
    theta4::Float64
end

function get_mid_status(p::StateSnake2D, q::StateSnake2D)::StateSnake2D
    return StateSnake2D(
        (p.x + q.x) / 2,
        (p.y + q.y) / 2,
        diff_angles(q.theta1, p.theta1) / 2 + p.theta1,
        diff_angles(q.theta2, p.theta2) / 2 + p.theta2,
        diff_angles(q.theta3, p.theta3) / 2 + p.theta3,
        diff_angles(q.theta4, p.theta4) / 2 + p.theta4,
    )
end

function dist(q1::StateSnake2D, q2::StateSnake2D)::Float64
    return norm([
        q1.x - q2.x,
        q1.y - q2.y,
        diff_angles(q1.theta1, q2.theta1) / π,
        diff_angles(q1.theta2, q2.theta2) / π,
        diff_angles(q1.theta3, q2.theta3) / π,
        diff_angles(q1.theta4, q2.theta4) / π,
    ])
end

function get_snake_positions(q::StateSnake2D, rad::Float64)::Vector{Vector{Float64}}
    p1 = [q.x, q.y]
    p2 = p1 + [cos(q.theta1), sin(q.theta1)] * rad
    p3 = p2 + [cos(q.theta2), sin(q.theta2)] * rad
    p4 = p3 + [cos(q.theta3), sin(q.theta3)] * rad
    p5 = p4 + [cos(q.theta4), sin(q.theta4)] * rad

    return [p1, p2, p3, p4, p5]
end

function check_self_collision_snake2d(
    P::Vector{Vector{Float64}},
    safety_dist::Float64 = SAFETY_DIST_LINE,
)::Bool
    any(
        e -> dist(e...) < safety_dist,
        [
            [P[1], P[2], P[3], P[4]],
            [P[1], P[2], P[4], P[5]],
            [P[2], P[3], P[4], P[5]],
            [P[3], P[4], P[5]],
        ],
    )
end

function gen_connect(
    q::StateSnake2D,
    obstacles::Vector{CircleObstacle2D},
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST_SNAKE2D,
    max_dist::Union{Nothing,Float64} = nothing,
    safety_dist::Float64 = SAFETY_DIST_LINE,
)::Function

    # check: q \in C_free
    f(q::StateSnake2D, i::Int64)::Bool = begin
        P = get_snake_positions(q, rads[i])

        any(x -> (x < 0 || 1 < x), vcat(P...)) && return false

        # obstacle
        for k = 2:5
            any(o -> dist(P[k-1], P[k], [o.x, o.y]) < o.r, obstacles) && return false
        end

        # self collision
        check_self_collision_snake2d(P, safety_dist) && return false

        return true
    end

    f(q_from::StateSnake2D, q_to::StateSnake2D, i::Int64)::Bool = begin
        D = dist(q_from, q_to)

        !isnothing(max_dist) && D > max_dist && return false

        dt1 = diff_angles(q_to.theta1, q_from.theta1)
        dt2 = diff_angles(q_to.theta2, q_from.theta2)
        dt3 = diff_angles(q_to.theta3, q_from.theta3)
        dt4 = diff_angles(q_to.theta4, q_from.theta4)

        for e in vcat(collect(0:step_dist:D) / D, 1.0)
            # angles
            T = [
                q_from.theta1 + e * dt1,
                q_from.theta2 + e * dt2,
                q_from.theta3 + e * dt3,
                q_from.theta4 + e * dt4,
            ]
            # positions
            P = [(1 - e) * [q_from.x, q_from.y] + e * [q_to.x, q_to.y]]
            foreach(k -> push!(P, P[k-1] + rads[i] * [cos(T[k-1]), sin(T[k-1])]), 2:5)

            # outside
            any(x -> (x < 0 || 1 < x), vcat(P...)) && return false

            # obstacles
            for k = 2:5
                any(o -> dist(P[k-1], P[k], [o.x, o.y]) < o.r, obstacles) && return false
            end

            # self collision
            check_self_collision_snake2d(P, safety_dist) && return false
        end

        return true
    end
    return f
end

function gen_collide(
    q::StateSnake2D,
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST_SNAKE2D,
    safety_dist::Float64 = SAFETY_DIST_LINE,
)::Function

    N = length(rads)

    f(
        q_i_from::StateSnake2D,
        q_i_to::StateSnake2D,
        q_j_from::StateSnake2D,
        q_j_to::StateSnake2D,
        i::Int64,
        j::Int64,
    ) = begin
        # check each pair of step
        D_i = dist(q_i_from, q_i_to)
        D_j = dist(q_j_from, q_j_to)

        dt_i = [
            diff_angles(q_i_to.theta1, q_i_from.theta1),
            diff_angles(q_i_to.theta2, q_i_from.theta2),
            diff_angles(q_i_to.theta3, q_i_from.theta3),
            diff_angles(q_i_to.theta4, q_i_from.theta4),
        ]

        dt_j = [
            diff_angles(q_j_to.theta1, q_j_from.theta1),
            diff_angles(q_j_to.theta2, q_j_from.theta2),
            diff_angles(q_j_to.theta3, q_j_from.theta3),
            diff_angles(q_j_to.theta4, q_j_from.theta4),
        ]

        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # intermediate angles & positions for agent-i
            T_i = [
                q_i_from.theta1 + e_i * dt_i[1],
                q_i_from.theta2 + e_i * dt_i[2],
                q_i_from.theta3 + e_i * dt_i[3],
                q_i_from.theta4 + e_i * dt_i[4],
            ]
            P_i = [(1 - e_i) * [q_i_from.x, q_i_from.y] + e_i * [q_i_to.x, q_i_to.y]]
            foreach(
                k -> push!(P_i, P_i[k-1] + rads[i] * [cos(T_i[k-1]), sin(T_i[k-1])]),
                2:5,
            )

            for e_j in vcat(collect(0:step_dist:D_j) / D_j, 1.0)
                # intermediate angles & positions for agent-j
                T_j = [
                    q_j_from.theta1 + e_j * dt_j[1],
                    q_j_from.theta2 + e_j * dt_j[2],
                    q_j_from.theta3 + e_j * dt_j[3],
                    q_j_from.theta4 + e_j * dt_j[4],
                ]
                P_j =
                    [(1 - e_j) * [q_j_from.x, q_j_from.y] + e_j * [q_j_to.x, q_j_to.y]]
                foreach(
                    k ->
                        push!(P_j, P_j[k-1] + rads[j] * [cos(T_j[k-1]), sin(T_j[k-1])]),
                    2:5,
                )

                # check_collision
                for k = 2:5, l = 2:5
                    dist(P_i[k-1], P_i[k], P_j[l-1], P_j[l]) < safety_dist && return true
                end
            end
        end
        return false
    end

    f(q_i::StateSnake2D, q_j::StateSnake2D, i::Int64, j::Int64) = begin
        P_i = get_snake_positions(q_i, rads[i])
        P_j = get_snake_positions(q_j, rads[j])

        # check_collision
        for k = 2:5, l = 2:5
            dist(P_i[k-1], P_i[k], P_j[l-1], P_j[l]) < safety_dist && return true
        end
        return false
    end

    f(Q_from::Vector{Node{StateSnake2D}}, Q_to::Vector{Node{StateSnake2D}}) = begin
        for i = 1:N, j = i+1:N
            if f(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q, i, j)
                return true
            end
        end
        return false
    end

    f(Q::Vector{Node{StateSnake2D}}, q_i_to::StateSnake2D, i::Int64) = begin
        q_i_from = Q[i].q
        D_i = dist(q_i_from, q_i_to)

        dt_i = [
            diff_angles(q_i_to.theta1, q_i_from.theta1),
            diff_angles(q_i_to.theta2, q_i_from.theta2),
            diff_angles(q_i_to.theta3, q_i_from.theta3),
            diff_angles(q_i_to.theta4, q_i_from.theta4),
        ]

        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # intermediate angles & positions for agent-i
            T_i = [
                q_i_from.theta1 + e_i * dt_i[1],
                q_i_from.theta2 + e_i * dt_i[2],
                q_i_from.theta3 + e_i * dt_i[3],
                q_i_from.theta4 + e_i * dt_i[4],
            ]
            P_i = [(1 - e_i) * [q_i_from.x, q_i_from.y] + e_i * [q_i_to.x, q_i_to.y]]
            foreach(
                k -> push!(P_i, P_i[k-1] + rads[i] * [cos(T_i[k-1]), sin(T_i[k-1])]),
                2:5,
            )

            for j in filter(k -> k != i, 1:N)
                P_j = get_snake_positions(Q[j].q, rads[j])

                # check collision
                for k = 2:5, l = 2:5
                    dist(P_i[k-1], P_i[k], P_j[l-1], P_j[l]) < safety_dist && return true
                end
            end
        end

        return false
    end

    return f
end

function gen_uniform_sampling(q::StateSnake2D)::Function
    () -> begin
        return StateSnake2D(rand(2)..., (rand(4) * 2π)...)
    end
end

function plot_motion!(q_from::StateSnake2D, q_to::StateSnake2D, rad::Float64, params)
    plot!([q_from.x, q_to.x], [q_from.y, q_to.y]; params...)
end

function plot_start_goal!(q_init::StateSnake2D, q_goal::StateSnake2D, rad::Float64, params)
    plot!([q_init.x], [q_init.y]; markershape = :hex, params...)
    plot!([q_goal.x], [q_goal.y]; markershape = :star, params...)
end

function plot_agent!(q::StateSnake2D, rad::Float64, color::String)
    P = get_snake_positions(q, rad)
    plot!(
        map(pos -> pos[1], P),
        map(pos -> pos[2], P),
        color = color,
        lw = 5,
        markershape = :circle,
        markersize = 3,
        label = nothing,
    )
    scatter!(
        [q.x],
        [q.y],
        markershpa = 10,
        markershape = :rect,
        color = color,
        label = nothing,
    )
end

function gen_random_instance_StateSnake2D(; params...)
    return gen_random_instance(StateSnake2D(0, 0, 0, 0, 0, 0); params...)
end

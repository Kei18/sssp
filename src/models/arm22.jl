struct StateArm22 <: AbsState
    theta1::Float64
    theta2::Float64
end

const STEP_DIST_ARM22 = 0.01

to_string(s::StateArm22) = @sprintf("(theta1: %.4f, theta2: %.4f)", s.theta1, s.theta2)

function get_mid_status(p::StateArm22, q::StateArm22)::StateArm22
    return StateArm22(
        diff_angles(q.theta1, p.theta1) / 2 + p.theta1,
        diff_angles(q.theta2, p.theta2) / 2 + p.theta2,
    )
end

function dist(q1::StateArm22, q2::StateArm22)
    return norm([
        diff_angles(q1.theta1, q2.theta1) / π,
        diff_angles(q1.theta2, q2.theta2) / π,
    ])
end

function get_arm22_positions(q::StateArm22, pos::Vector{Float64}, rad::Float64)
    a = pos
    b = a + [cos(q.theta1), sin(q.theta1)] * rad
    c = b + [cos(q.theta2), sin(q.theta2)] * rad
    return [a, b, c]
end

function gen_connect(
    q::StateArm22,
    obstacles::Vector{CircleObstacle2D},
    positions::Vector{Vector{Float64}},
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST,
    max_dist::Union{Nothing,Float64} = nothing,
    safety_dist::Float64 = SAFETY_DIST_LINE,
)::Function

    # check: q \in C_free
    f(q::StateArm22, i::Int64)::Bool = begin
        a, b, c = get_arm22_positions(q, positions[i], rads[i])

        any(x -> (x < 0 || 1 < x), vcat(a, b, c)) && return false

        any(
            o -> (dist(a, b, [o.x, o.y]) < o.r || dist(b, c, [o.x, o.y]) < o.r),
            obstacles,
        ) && return false

        # self collision
        dist(a, b, c) < safety_dist && return false

        return true
    end

    f(q_from::StateArm22, q_to::StateArm22, i::Int64)::Bool = begin
        D = dist(q_from, q_to)
        !isnothing(max_dist) && D > max_dist && return false

        dt1 = diff_angles(q_to.theta1, q_from.theta1)
        dt2 = diff_angles(q_to.theta2, q_from.theta2)

        a = positions[i]

        for e in vcat(collect(0:step_dist:D) / D, 1.0)
            t1 = q_from.theta1 + e * dt1
            t2 = q_from.theta2 + e * dt2

            b = a + rads[i] * [cos(t1), sin(t1)]
            c = b + rads[i] * [cos(t2), sin(t2)]

            # outside
            any(x -> (x < 0 || 1 < x), vcat(a, b, c)) && return false

            # obstacles
            any(
                o -> (dist(a, b, [o.x, o.y]) < o.r || dist(b, c, [o.x, o.y]) < o.r),
                obstacles,
            ) && return false

            # self collision
            dist(a, b, c) < safety_dist && return false
        end

        return true
    end
    return f
end

function gen_collide(
    q::StateArm22,
    positions::Vector{Vector{Float64}},
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST,
    safety_dist::Float64 = SAFETY_DIST_LINE,
)::Function

    N = length(rads)

    f(
        q_i_from::StateArm22,
        q_i_to::StateArm22,
        q_j_from::StateArm22,
        q_j_to::StateArm22,
        i::Int64,
        j::Int64,
    ) = begin
        # check each pair of step
        D_i = dist(q_i_from, q_i_to)
        D_j = dist(q_j_from, q_j_to)

        dt1_i = diff_angles(q_i_to.theta1, q_i_from.theta1)
        dt2_i = diff_angles(q_i_to.theta2, q_i_from.theta2)
        dt1_j = diff_angles(q_j_to.theta1, q_j_from.theta1)
        dt2_j = diff_angles(q_j_to.theta2, q_j_from.theta2)

        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # intermediate angles & positions for agent-i
            t1_i = q_i_from.theta1 + e_i * dt1_i
            t2_i = q_i_from.theta2 + e_i * dt2_i
            a_i = positions[i]
            b_i = rads[i] * [cos(t1_i), sin(t1_i)] + a_i
            c_i = rads[i] * [cos(t2_i), sin(t2_i)] + b_i

            for e_j in vcat(collect(0:step_dist:D_j) / D_j, 1.0)
                # intermediate angles & positions for agent-j
                t1_j = q_j_from.theta1 + e_j * dt1_j
                t2_j = q_j_from.theta2 + e_j * dt2_j
                a_j = positions[j]
                b_j = rads[j] * [cos(t1_j), sin(t1_j)] + a_j
                c_j = rads[j] * [cos(t2_j), sin(t2_j)] + b_j

                # check_collision
                any(
                    e -> dist(e...) < safety_dist,
                    [
                        [a_i, b_i, a_j, b_j],
                        [a_i, b_i, b_j, c_j],
                        [b_i, c_i, a_j, b_j],
                        [b_i, c_i, b_j, c_j],
                    ],
                ) && return true
            end
        end
        return false
    end

    f(q_i::StateArm22, q_j::StateArm22, i::Int64, j::Int64) = begin
        a_i, b_i, c_i = get_arm22_positions(q_i, positions[i], rads[i])
        a_j, b_j, c_j = get_arm22_positions(q_j, positions[j], rads[j])

        # check_collision
        return any(
            e -> dist(e...) < safety_dist,
            [
                [a_i, b_i, a_j, b_j],
                [a_i, b_i, b_j, c_j],
                [b_i, c_i, a_j, b_j],
                [b_i, c_i, b_j, c_j],
            ],
        )
    end

    f(Q_from::Vector{Node{StateArm22}}, Q_to::Vector{Node{StateArm22}}) = begin
        for i = 1:N, j = i+1:N
            if f(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q, i, j)
                return true
            end
        end
        return false
    end

    f(Q::Vector{Node{StateArm22}}, q_i_to::StateArm22, i::Int64) = begin
        q_i_from = Q[i].q
        D_i = dist(q_i_from, q_i_to)

        dt1_i = diff_angles(q_i_to.theta1, q_i_from.theta1)
        dt2_i = diff_angles(q_i_to.theta2, q_i_from.theta2)

        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # intermediate angles & positions for agent-i
            t1_i = q_i_from.theta1 + e_i * dt1_i
            t2_i = q_i_from.theta2 + e_i * dt2_i
            a_i = positions[i]
            b_i = rads[i] * [cos(t1_i), sin(t1_i)] + a_i
            c_i = rads[i] * [cos(t2_i), sin(t2_i)] + b_i

            for j in filter(k -> k != i, 1:N)
                a_j, b_j, c_j = get_arm22_positions(Q[j].q, positions[j], rads[j])

                # check collision
                any(
                    e -> dist(e...) < safety_dist,
                    [
                        [a_i, b_i, a_j, b_j],
                        [a_i, b_i, b_j, c_j],
                        [b_i, c_i, a_j, b_j],
                        [b_i, c_i, b_j, c_j],
                    ],
                ) && return true
            end
        end

        return false
    end

    return f
end

function gen_uniform_sampling(q::StateArm22)::Function
    () -> begin
        return StateArm22((rand(2) * 2π)...)
    end
end

function plot_motion!(
    q_from::StateArm22,
    q_to::StateArm22,
    pos::Vector{Float64},
    rad::Float64,
    params,
)
    p1 = get_arm22_positions(q_from, pos, rad)[end]
    p2 = get_arm22_positions(q_to, pos, rad)[end]
    plot!([p1[1], p2[1]], [p1[2], p2[2]]; params...)
end

function plot_start_goal!(
    q_init::StateArm22,
    q_goal::StateArm22,
    pos::Vector{Float64},
    rad::Float64,
    params,
)
    pos_init = get_arm22_positions(q_init, pos, rad)[end]
    pos_goal = get_arm22_positions(q_goal, pos, rad)[end]
    plot!([pos_init[1]], [pos_init[2]]; markershape = :hex, params...)
    plot!([pos_goal[1]], [pos_goal[2]]; markershape = :star, params...)
end

function plot_agent!(q::StateArm22, pos::Vector{Float64}, rad::Float64, color::String)
    _, pos1, pos2 = get_arm22_positions(q, pos, rad)
    plot!(
        [pos[1], pos1[1], pos2[1]],
        [pos[2], pos1[2], pos2[2]],
        color = color,
        lw = 5,
        markershape = :circle,
        markersize = 3,
        label = nothing,
    )
    scatter!(
        [pos[1]],
        [pos[2]],
        markershpa = 10,
        markershape = :rect,
        color = color,
        label = nothing,
    )
end

function gen_random_instance_StateArm22(;
    N_min::Int64 = 2,
    N_max::Int64 = 8,
    N::Int64 = rand(N_min:N_max),
    num_obs_min::Int64 = 0,
    num_obs_max::Int64 = 10,
    num_obs::Int64 = rand(num_obs_min:num_obs_max),
    rad::Float64 = 0.025,
    rad_obs::Float64 = 0.05,
    rad_min::Float64 = rad,
    rad_max::Float64 = rad,
    rad_obs_min::Float64 = rad_obs,
    rad_obs_max::Float64 = rad_obs,
    TIME_LIMIT::Float64 = 0.5,
)::Tuple{
    Vector{StateArm22},  # initial configuration
    Vector{StateArm22},  # goal configuration
    Vector{CircleObstacle2D},
    Vector{Vector{Float64}},  # positions
    Vector{Float64},  # radius
}

    while true
        t_s = now()
        timeover() = elapsed_sec(t_s) > TIME_LIMIT

        # generate obstacles
        obstacles = gen_obstacles(2, num_obs, rad_obs_min, rad_obs_max)

        # determine positions
        positions = map(e -> rand(2), 1:N)

        # determine rads
        rads = map(e -> rand() * (rad_max - rad_min) + rad_min, 1:N)

        # generate starts & goals
        _q = StateArm22(0, 0)
        connect = gen_connect(_q, obstacles, positions, rads)
        collide = gen_collide(_q, positions, rads)
        config_init, config_goal = gen_config_init_goal(_q, N, connect, collide, timeover)

        timeover() && continue
        return (config_init, config_goal, obstacles, positions, rads)
    end
end

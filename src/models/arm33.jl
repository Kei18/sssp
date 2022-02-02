"""model definition of arm33"""

struct StateArm33 <: AbsState
    θ1::Float64
    ϕ1::Float64
    θ2::Float64
    ϕ2::Float64
    θ3::Float64
    ϕ3::Float64
end

function get_mid_status(p::StateArm33, q::StateArm33)::StateArm33
    return StateArm33(
        diff_angles(q.θ1, p.θ1) / 2 + p.θ1,
        diff_angles(q.ϕ1, p.ϕ1) / 2 + p.ϕ1,
        diff_angles(q.θ2, p.θ2) / 2 + p.θ2,
        diff_angles(q.ϕ2, p.ϕ2) / 2 + p.ϕ2,
        diff_angles(q.θ3, p.θ3) / 2 + p.θ3,
        diff_angles(q.ϕ3, p.ϕ3) / 2 + p.ϕ3,
    )
end

function dist(q1::StateArm33, q2::StateArm33)
    return norm([
        diff_angles(q1.θ1, q2.θ1) / π,
        diff_angles(q1.ϕ1, q2.ϕ1) / π,
        diff_angles(q1.θ2, q2.θ2) / π,
        diff_angles(q1.ϕ2, q2.ϕ2) / π,
        diff_angles(q1.θ3, q2.θ3) / π,
        diff_angles(q1.ϕ3, q2.ϕ3) / π,
    ])
end

function get_arm33_positions(q::StateArm33, pos::Vector{Float64}, rad::Float64)
    a = pos
    b = a + [sin(q.θ1) * cos(q.ϕ1), sin(q.θ1) * sin(q.ϕ1), cos(q.θ1)] * rad
    c = b + [sin(q.θ2) * cos(q.ϕ2), sin(q.θ2) * sin(q.ϕ2), cos(q.θ2)] * rad
    d = c + [sin(q.θ3) * cos(q.ϕ3), sin(q.θ3) * sin(q.ϕ3), cos(q.θ3)] * rad
    return [a, b, c, d]
end

function gen_connect(
    q::StateArm33,
    obstacles::Vector{CircleObstacle3D},
    positions::Vector{Vector{Float64}},
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST,
    max_dist::Union{Nothing,Float64} = nothing,
    safety_dist::Float64 = SAFETY_DIST_LINE,
)::Function

    # check: q \in C_free
    f(q::StateArm33, i::Int64)::Bool = begin
        P = get_arm33_positions(q, positions[i], rads[i])

        any(x -> (x < 0 || 1 < x), vcat(P...)) && return false

        # obstacle
        for k = 2:4
            any(o -> dist(P[k-1], P[k], [o.x, o.y, o.z]) < o.r, obstacles) && return false
        end

        # self collision
        dist(P[1], P[2], P[3], P[4]) < safety_dist && return false

        return true
    end

    f(q_from::StateArm33, q_to::StateArm33, i::Int64)::Bool = begin
        D = dist(q_from, q_to)
        !isnothing(max_dist) && D > max_dist && return false

        dt1_θ = diff_angles(q_to.θ1, q_from.θ1)
        dt1_ϕ = diff_angles(q_to.ϕ1, q_from.ϕ1)
        dt2_θ = diff_angles(q_to.θ2, q_from.θ2)
        dt2_ϕ = diff_angles(q_to.ϕ2, q_from.ϕ2)
        dt3_θ = diff_angles(q_to.θ3, q_from.θ3)
        dt3_ϕ = diff_angles(q_to.ϕ3, q_from.ϕ3)

        for e in vcat(collect(0:step_dist:D) / D, 1.0)
            # angles
            arr_θ = [q_from.θ1 + e * dt1_θ, q_from.θ2 + e * dt2_θ, q_from.θ3 + e * dt3_θ]
            arr_ϕ =
                [q_from.ϕ1 + e * dt1_ϕ, q_from.ϕ2 + e * dt2_ϕ, q_from.ϕ3 + e * dt3_ϕ]
            # positions
            P = [positions[i]]
            for k = 2:4
                push!(
                    P,
                    P[k-1] +
                    rads[i] * [
                        sin(arr_θ[k-1]) * cos(arr_ϕ[k-1]),
                        sin(arr_θ[k-1]) * sin(arr_ϕ[k-1]),
                        cos(arr_θ[k-1]),
                    ],
                )
            end

            # outside
            any(x -> (x < 0 || 1 < x), vcat(P...)) && return false

            # obstacles
            for k = 2:4
                any(o -> dist(P[k-1], P[k], [o.x, o.y, o.z]) < o.r, obstacles) &&
                    return false
            end

            # self collision
            dist(P[1], P[2], P[3], P[4]) < safety_dist && return false
        end

        return true
    end
    return f
end

function gen_collide(
    q::StateArm33,
    positions::Vector{Vector{Float64}},
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST,
    safety_dist::Float64 = SAFETY_DIST_LINE,
)::Function

    N = length(rads)

    f(
        q_i_from::StateArm33,
        q_i_to::StateArm33,
        q_j_from::StateArm33,
        q_j_to::StateArm33,
        i::Int64,
        j::Int64,
    ) = begin
        # check each pair of step
        D_i = dist(q_i_from, q_i_to)
        D_j = dist(q_j_from, q_j_to)

        dt1_θ_i = diff_angles(q_i_to.θ1, q_i_from.θ1)
        dt1_ϕ_i = diff_angles(q_i_to.ϕ1, q_i_from.ϕ1)
        dt2_θ_i = diff_angles(q_i_to.θ2, q_i_from.θ2)
        dt2_ϕ_i = diff_angles(q_i_to.ϕ2, q_i_from.ϕ2)
        dt3_θ_i = diff_angles(q_i_to.θ3, q_i_from.θ3)
        dt3_ϕ_i = diff_angles(q_i_to.ϕ3, q_i_from.ϕ3)

        dt1_θ_j = diff_angles(q_j_to.θ1, q_j_from.θ1)
        dt1_ϕ_j = diff_angles(q_j_to.ϕ1, q_j_from.ϕ1)
        dt2_θ_j = diff_angles(q_j_to.θ2, q_j_from.θ2)
        dt2_ϕ_j = diff_angles(q_j_to.ϕ2, q_j_from.ϕ2)
        dt3_θ_j = diff_angles(q_j_to.θ3, q_j_from.θ3)
        dt3_ϕ_j = diff_angles(q_j_to.ϕ3, q_j_from.ϕ3)

        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # intermediate angles & positions for agent-i
            arr_θ_i = [
                q_i_from.θ1 + e_i * dt1_θ_i,
                q_i_from.θ2 + e_i * dt2_θ_i,
                q_i_from.θ3 + e_i * dt3_θ_i,
            ]
            arr_ϕ_i = [
                q_i_from.ϕ1 + e_i * dt1_ϕ_i,
                q_i_from.ϕ2 + e_i * dt2_ϕ_i,
                q_i_from.ϕ3 + e_i * dt3_ϕ_i,
            ]
            # positions
            P_i = [positions[i]]
            for k = 2:4
                push!(
                    P_i,
                    P_i[k-1] +
                    rads[i] * [
                        sin(arr_θ_i[k-1]) * cos(arr_ϕ_i[k-1]),
                        sin(arr_θ_i[k-1]) * sin(arr_ϕ_i[k-1]),
                        cos(arr_θ_i[k-1]),
                    ],
                )
            end

            for e_j in vcat(collect(0:step_dist:D_j) / D_j, 1.0)
                # intermediate angles & positions for agent-j
                arr_θ_j = [
                    q_j_from.θ1 + e_j * dt1_θ_j,
                    q_j_from.θ2 + e_j * dt2_θ_j,
                    q_j_from.θ3 + e_j * dt3_θ_j,
                ]
                arr_ϕ_j = [
                    q_j_from.ϕ1 + e_j * dt1_ϕ_j,
                    q_j_from.ϕ2 + e_j * dt2_ϕ_j,
                    q_j_from.ϕ3 + e_j * dt3_ϕ_j,
                ]
                # positions
                P_j = [positions[j]]
                for k = 2:4
                    push!(
                        P_j,
                        P_j[k-1] +
                        rads[j] * [
                            sin(arr_θ_j[k-1]) * cos(arr_ϕ_j[k-1]),
                            sin(arr_θ_j[k-1]) * sin(arr_ϕ_j[k-1]),
                            cos(arr_θ_j[k-1]),
                        ],
                    )
                end

                # check_collision
                for k = 2:4, l = 2:4
                    dist(P_i[k-1], P_i[k], P_j[l-1], P_j[l]) < safety_dist && return true
                end
            end
        end
        return false
    end

    f(q_i::StateArm33, q_j::StateArm33, i::Int64, j::Int64) = begin
        P_i = get_arm33_positions(q_i, positions[i], rads[i])
        P_j = get_arm33_positions(q_j, positions[j], rads[j])

        # check_collision
        for k = 2:4, l = 2:4
            dist(P_i[k-1], P_i[k], P_j[l-1], P_j[l]) < safety_dist && return true
        end
        return false
    end

    f(Q_from::Vector{Node{StateArm33}}, Q_to::Vector{Node{StateArm33}}) = begin
        for i = 1:N, j = i+1:N
            if f(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q, i, j)
                return true
            end
        end
        return false
    end

    f(Q::Vector{Node{StateArm33}}, q_i_to::StateArm33, i::Int64) = begin
        q_i_from = Q[i].q
        D_i = dist(q_i_from, q_i_to)

        dt1_θ_i = diff_angles(q_i_to.θ1, q_i_from.θ1)
        dt1_ϕ_i = diff_angles(q_i_to.ϕ1, q_i_from.ϕ1)
        dt2_θ_i = diff_angles(q_i_to.θ2, q_i_from.θ2)
        dt2_ϕ_i = diff_angles(q_i_to.ϕ2, q_i_from.ϕ2)
        dt3_θ_i = diff_angles(q_i_to.θ3, q_i_from.θ3)
        dt3_ϕ_i = diff_angles(q_i_to.ϕ3, q_i_from.ϕ3)

        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # intermediate angles & positions for agent-i
            arr_θ_i = [
                q_i_from.θ1 + e_i * dt1_θ_i,
                q_i_from.θ2 + e_i * dt2_θ_i,
                q_i_from.θ3 + e_i * dt3_θ_i,
            ]
            arr_ϕ_i = [
                q_i_from.ϕ1 + e_i * dt1_ϕ_i,
                q_i_from.ϕ2 + e_i * dt2_ϕ_i,
                q_i_from.ϕ3 + e_i * dt3_ϕ_i,
            ]
            # positions
            P_i = [positions[i]]
            for k = 2:4
                push!(
                    P_i,
                    P_i[k-1] +
                    rads[i] * [
                        sin(arr_θ_i[k-1]) * cos(arr_ϕ_i[k-1]),
                        sin(arr_θ_i[k-1]) * sin(arr_ϕ_i[k-1]),
                        cos(arr_θ_i[k-1]),
                    ],
                )
            end

            for j in filter(k -> k != i, 1:N)
                P_j = get_arm33_positions(Q[j].q, positions[j], rads[j])

                # check collision
                for k = 2:4, l = 2:4
                    dist(P_i[k-1], P_i[k], P_j[l-1], P_j[l]) < safety_dist && return true
                end
            end
        end

        return false
    end

    return f
end

function gen_uniform_sampling(q::StateArm33)::Function
    () -> begin
        return StateArm33((rand(6) * 2π)...)
    end
end

function plot_motion!(
    q_from::StateArm33,
    q_to::StateArm33,
    pos::Vector{Float64},
    rad::Float64,
    params,
)
    p1 = get_arm33_positions(q_from, pos, rad)[end]
    p2 = get_arm33_positions(q_to, pos, rad)[end]
    plot!([p1[1], p2[1]], [p1[2], p2[2]], [p1[3], p2[3]]; params...)
end

function plot_start_goal!(
    q_init::StateArm33,
    q_goal::StateArm33,
    pos::Vector{Float64},
    rad::Float64,
    params,
)
    pos_init = get_arm33_positions(q_init, pos, rad)[end]
    pos_goal = get_arm33_positions(q_goal, pos, rad)[end]
    plot!([pos_init[1]], [pos_init[2]], [pos_init[3]]; markershape = :hex, params...)
    plot!([pos_goal[1]], [pos_goal[2]], [pos_goal[3]]; markershape = :star, params...)
end

function plot_agent!(q::StateArm33, pos::Vector{Float64}, rad::Float64, color::String)
    P = get_arm33_positions(q, pos, rad)
    plot!(
        map(p -> p[1], P),
        map(p -> p[2], P),
        map(p -> p[3], P),
        color = color,
        lw = 5,
        markershape = :circle,
        markersize = 3,
        label = nothing,
    )
    scatter!(
        [pos[1]],
        [pos[2]],
        [pos[3]],
        markersize = 10,
        markershape = :rect,
        color = color,
        label = nothing,
    )
end

function gen_random_instance_StateArm33(;
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
    Vector{StateArm33},  # initial configuration
    Vector{StateArm33},  # goal configuration
    Vector{CircleObstacle3D},
    Vector{Vector{Float64}},  # positions
    Vector{Float64},  # radius
}

    _q = StateArm33(0, 0, 0, 0, 0, 0)
    while true
        t_s = now()
        timeover() = elapsed_sec(t_s) > TIME_LIMIT

        # generate obstacles
        obstacles = gen_obstacles(3, num_obs, rad_obs_min, rad_obs_max)

        # determine positions
        positions = map(e -> rand(3), 1:N)

        # determine rads
        rads = map(e -> rand() * (rad_max - rad_min) + rad_min, 1:N)

        # generate starts & goals
        connect = gen_connect(_q, obstacles, positions, rads)
        collide = gen_collide(_q, positions, rads)
        config_init, config_goal = gen_config_init_goal(_q, N, connect, collide, timeover)

        timeover() && continue
        return (config_init, config_goal, obstacles, positions, rads)
    end
end

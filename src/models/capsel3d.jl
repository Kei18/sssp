"""model definition of capsel3d"""

const STEP_DIST_CAPSEL3D = 0.05

struct StateCapsel3D <: AbsState
    x::Float64
    y::Float64
    z::Float64
    ϕ::Float64  # roll
    θ::Float64  # pitch
    ψ::Float64  # yaw
end

function get_mid_status(p::StateCapsel3D, q::StateCapsel3D)::StateCapsel3D
    return StateCapsel3D(
        (p.x + q.x) / 2,
        (p.y + q.y) / 2,
        (p.z + q.z) / 2,
        diff_angles(q.ϕ, p.ϕ) / 2 + p.ϕ,
        diff_angles(q.θ, p.θ) / 2 + p.θ,
        diff_angles(q.ψ, p.ψ) / 2 + p.ψ,
    )
end

function dist(q1::StateCapsel3D, q2::StateCapsel3D)::Float64
    # normalize
    return norm([
        q1.x - q2.x,
        q1.y - q2.y,
        q1.z - q2.z,
        diff_angles(q1.ϕ, q2.ϕ) / π,
        diff_angles(q1.θ, q2.θ) / π,
        diff_angles(q1.ψ, q2.ψ) / π,
    ])
end

"""get rotation matrix for parameters roll, pitch, and yaw"""
function get_rotation_matrix(
    ϕ::Float64,  # roll
    θ::Float64,  # pitch
    ψ::Float64,   # yaw
)::Matrix{Float64}
    cϕ, sϕ = cos(ϕ), sin(ϕ)
    cθ, sθ = cos(θ), sin(θ)
    cψ, sψ = cos(ψ), sin(ψ)
    # Note: the last two columns are not use in Capsel3D model
    return [
        (cϕ*cθ) (-sϕ*cψ+cϕ*sθ*sψ) (sϕ*sψ+cϕ*sθ*cψ)
        (sϕ*cθ) (cϕ*cψ+sϕ*sθ*sψ) (-cϕ*sψ+sϕ*sθ*cψ)
        (-sθ) (cθ*sψ) (cθ*cψ)
    ]
end

function get_capsel3d_points(q::StateCapsel3D, axes::Float64)::Vector{Vector{Float64}}
    R = get_rotation_matrix(q.ϕ, q.θ, q.ψ)
    p1 = [q.x, q.y, q.z]
    p2 = R * [axes, 0, 0] + p1
    return [p1, p2]
end

function gen_connect(
    q::StateCapsel3D,  # to identify type
    obstacles::Vector{CircleObstacle3D},
    rads::Vector{Float64},
    axises::Vector{Float64};
    step_dist::Float64 = STEP_DIST_CAPSEL3D,
    max_dist::Union{Nothing,Float64} = nothing,
)::Function

    # check: q \in C_free
    f(q::StateCapsel3D, i::Int64)::Bool = begin
        a, b = get_capsel3d_points(q, axises[i])
        # outside
        any(x -> (x < rads[i] || 1 - rads[i] < x), vcat(a, b)) && return false
        # obstacles
        any(o -> dist(a, b, [o.x, o.y, o.z]) < o.r + rads[i], obstacles) && return false
        return true
    end

    f(q_from::StateCapsel3D, q_to::StateCapsel3D, i::Int64)::Bool = begin
        D = dist(q_from, q_to)
        !isnothing(max_dist) && D > max_dist && return false

        dϕ = diff_angles(q_to.ϕ, q_from.ϕ)
        dθ = diff_angles(q_to.θ, q_from.θ)
        dψ = diff_angles(q_to.ψ, q_from.ψ)

        for e in vcat(collect(0:step_dist:D) / D, 1.0)
            # intermediate: root
            a = (1 - e) * [q_from.x, q_from.y, q_from.z] + e * [q_to.x, q_to.y, q_to.z]
            # intermediate: rotation matrix
            R = get_rotation_matrix(q_from.ϕ + e * dϕ, q_from.θ + e * dθ, q_from.ψ + e * dψ)
            # intermediate: tip
            b = a + R * [axises[i], 0, 0]
            # outside
            any(x -> (x < rads[i] || 1 - rads[i] < x), vcat(a, b)) && return false
            # obstacles
            any(o -> dist(a, b, [o.x, o.y, o.z]) < o.r + rads[i], obstacles) &&
                return false
        end

        return true
    end

    return f
end


function gen_collide(
    q::StateCapsel3D,
    rads::Vector{Float64},
    axises::Vector{Float64};
    step_dist::Float64 = STEP_DIST_CAPSEL3D,
)::Function

    N = length(rads)

    f(
        q_i_from::StateCapsel3D,
        q_i_to::StateCapsel3D,
        q_j_from::StateCapsel3D,
        q_j_to::StateCapsel3D,
        i::Int64,
        j::Int64,
    ) = begin
        # check each pair of step
        D_i = dist(q_i_from, q_i_to)
        D_j = dist(q_j_from, q_j_to)

        dϕ_i = diff_angles(q_i_to.ϕ, q_i_from.ϕ)
        dθ_i = diff_angles(q_i_to.θ, q_i_from.θ)
        dψ_i = diff_angles(q_i_to.ψ, q_i_from.ψ)

        dϕ_j = diff_angles(q_j_to.ϕ, q_j_from.ϕ)
        dθ_j = diff_angles(q_j_to.θ, q_j_from.θ)
        dψ_j = diff_angles(q_j_to.ψ, q_j_from.ψ)

        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # root, rotation, tip
            a_i =
                (1 - e_i) * [q_i_from.x, q_i_from.y, q_i_from.z] +
                e_i * [q_i_to.x, q_i_to.y, q_i_to.z]
            R_i = get_rotation_matrix(
                q_i_from.ϕ + e_i * dϕ_i,
                q_i_from.θ + e_i * dθ_i,
                q_i_from.ψ + e_i * dψ_i,
            )
            b_i = a_i + R_i * [axises[i], 0, 0]

            for e_j in vcat(collect(0:step_dist:D_j) / D_j, 1.0)
                # root, rotation, tip
                a_j =
                    (1 - e_j) * [q_j_from.x, q_j_from.y, q_j_from.z] +
                    e_j * [q_j_to.x, q_j_to.y, q_j_to.z]
                R_j = get_rotation_matrix(
                    q_j_from.ϕ + e_j * dϕ_j,
                    q_j_from.θ + e_j * dθ_j,
                    q_j_from.ψ + e_j * dψ_j,
                )
                b_j = a_j + R_j * [axises[j], 0, 0]

                # check collision
                dist(a_i, b_i, a_j, b_j) < rads[i] + rads[j] && return true
            end
        end
        return false
    end

    f(q_i::StateCapsel3D, q_j::StateCapsel3D, i::Int64, j::Int64) = begin
        a_i, b_i = get_capsel3d_points(q_i, axises[i])
        a_j, b_j = get_capsel3d_points(q_j, axises[j])
        return dist(a_i, b_i, a_j, b_j) < rads[i] + rads[j]
    end

    f(Q_from::Vector{Node{StateCapsel3D}}, Q_to::Vector{Node{StateCapsel3D}}) = begin
        for i = 1:N, j = i+1:N
            f(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q, i, j) && return true
        end
        return false
    end

    f(Q::Vector{Node{StateCapsel3D}}, q_i_to::StateCapsel3D, i::Int64) = begin
        q_i_from = Q[i].q
        D_i = dist(q_i_from, q_i_to)

        dϕ_i = diff_angles(q_i_to.ϕ, q_i_from.ϕ)
        dθ_i = diff_angles(q_i_to.θ, q_i_from.θ)
        dψ_i = diff_angles(q_i_to.ψ, q_i_from.ψ)

        positions = map(j -> get_capsel3d_points(Q[j].q, axises[j]), 1:N)
        for e_i in vcat(collect(0:step_dist:D_i) / D_i, 1.0)
            # root, rotation, tip
            a_i =
                (1 - e_i) * [q_i_from.x, q_i_from.y, q_i_from.z] +
                e_i * [q_i_to.x, q_i_to.y, q_i_to.z]
            R_i = get_rotation_matrix(
                q_i_from.ϕ + e_i * dϕ_i,
                q_i_from.θ + e_i * dθ_i,
                q_i_from.ψ + e_i * dψ_i,
            )
            b_i = a_i + R_i * [axises[i], 0, 0]

            for j in filter(k -> k != i, 1:N)
                # check collision
                dist(a_i, b_i, positions[j]...) < rads[i] + rads[j] && return true
            end
        end

        return false
    end

    return f
end

function gen_uniform_sampling(q::StateCapsel3D)::Function
    () -> begin
        return StateCapsel3D(rand(3)..., (rand(3) * 2π)...)
    end
end

function plot_motion!(
    q_from::StateCapsel3D,
    q_to::StateCapsel3D,
    rad::Float64,
    axes::Float64,
    params,
)
    plot!([q_from.x, q_to.x], [q_from.y, q_to.y], [q_from.z, q_to.z]; params...)
end

function plot_start_goal!(
    q_init::StateCapsel3D,
    q_goal::StateCapsel3D,
    rad::Float64,
    axes::Float64,
    params,
)
    plot!([q_init.x], [q_init.y], [q_init.z]; markershape = :hex, params...)
    plot!([q_goal.x], [q_goal.y], [q_goal.z]; markershape = :star, params...)
end

function plot_agent!(
    q::StateCapsel3D,
    rad::Float64,
    axes::Float64,
    color::String;
    step::Float64 = 0.2,
)
    R = get_rotation_matrix(q.ϕ, q.θ, q.ψ)
    a = [q.x, q.y, q.z]
    b = a + R * [axes, 0, 0]

    plotP(P) = begin
        plot!(
            map(e -> e[1], P),
            map(e -> e[2], P),
            map(e -> e[3], P),
            label = nothing,
            color = color,
        )
    end

    P1 = map(t -> a + R * [0, cos(t) * rad, sin(t) * rad], 0:step:2π)
    P2 = map(t -> b + R * [0, cos(t) * rad, sin(t) * rad], 0:step:2π)
    plotP(P1)
    plotP(P2)

    P1 = map(t -> a + R * [cos(t) * rad, sin(t) * rad, 0], π/2:step:3π/2)
    plotP(P1)
    P2 = map(t -> b + R * [cos(t) * rad, sin(t) * rad, 0], -π/2:step:π/2)
    plotP(P2)
    plotP([P1[1], P2[end]])
    plotP([P1[end], P2[1]])

    P1 = map(t -> a + R * [sin(t) * rad, 0, cos(t) * rad], π:step:2π)
    P2 = map(t -> b + R * [sin(t) * rad, 0, cos(t) * rad], 0:step:π)
    plotP(P1)
    plotP(P2)
    plotP([P1[1], P2[end]])
    plotP([P1[end], P2[1]])
end

function gen_random_instance_StateCapsel3D(;
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
    Vector{StateCapsel3D},  # initial configuration
    Vector{StateCapsel3D},  # goal configuration
    Vector{CircleObstacle3D},
    Vector{Float64},  # radius
    Vector{Float64},  # axises
}

    _q = StateCapsel3D(0, 0, 0, 0, 0, 0)
    while true
        t_s = now()
        timeover() = elapsed_sec(t_s) > TIME_LIMIT

        # generate obstacles
        obstacles = gen_obstacles(3, num_obs, rad_obs_min, rad_obs_max)

        # determine rads
        rads = map(e -> rand() * (rad_max - rad_min) + rad_min, 1:N)

        # determine axises
        axises = map(e -> rand() * (rad_max - rad_min) + rad_min, 1:N)

        # generate starts & goals
        connect = gen_connect(_q, obstacles, rads, axises)
        collide = gen_collide(_q, rads, axises)
        config_init, config_goal = gen_config_init_goal(_q, N, connect, collide, timeover)

        timeover() && continue
        return (config_init, config_goal, obstacles, rads, axises)
    end
end

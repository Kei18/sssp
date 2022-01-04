struct StateCar <: AbsState
    x::Float64
    y::Float64
    theta::Float64
end

to_string(s::StateCar) = @sprintf("(%.4f, %.4f, theta: %.4f)", s.x, s.y, s.theta)

const OMEGA_MAX = 4π

function dist(q1::StateCar, q2::StateCar)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y])
end

function get_omega_t(
    q_from::StateCar,
    q_to::StateCar,
    omega_max::Float64=OMEGA_MAX
    )::Tuple{Float64, Float64}

    theta_diff = q_to.theta - q_from.theta
    x_diff = q_to.x - q_from.x
    y_diff = q_to.y - q_from.y

    if theta_diff == 0
        omega = 0.0
        t = (cos(q_from.theta) != 0) ?
            (q_to.x - q_from.x) / cos(q_from.theta) :
            (q_to.y - q_from.y) / sin(q_from.theta)
        return (omega, t)
    elseif x_diff == 0 && y_diff == 0
        omega = omega_max
        t = abs(q_to.theta - q_from.theta) / omega_max
        return (omega, t)
    end

    get_omega_x = x_diff -> (sin(q_to.theta) - sin(q_from.theta)) / x_diff
    get_omega_y = y_diff -> (cos(q_from.theta) - cos(q_to.theta)) / y_diff
    get_t = omega -> (q_to.theta - q_from.theta) / omega

    if x_diff != 0 && y_diff != 0
        omega_x = get_omega_x(x_diff)
        omega_y = get_omega_y(y_diff)
        if !isapprox(mod(omega_x, 2π), mod(omega_y, 2π))
            return (0.0, -1.0)
        end
        t = get_t(omega_x)
        return (omega_x, t)
    elseif x_diff == 0
        omega = get_omega_y(y_diff)
        t = get_t(omega)
        return (omega, t)
    else
        # i.e., y_diff==0
        omega = get_omega_x(x_diff)
        t = get_t(omega)
        return (omega, t)
    end
end

function get_car_intermediate_state(
    q_from::StateCar,
    omega::Float64,
    t::Float64,
    e::Float64
    )::StateCar

    # e \in [0, 1]
    if isapprox([omega, t], [0, 0]); return q_from; end
    x = q_from.x + (sin(omega * e * t + q_from.theta) - sin(q_from.theta)) / omega
    y = q_from.y + (-cos(omega * e * t + q_from.theta) + cos(q_from.theta)) / omega
    theta = q_from.theta + t * omega
    return StateCar(x, y, theta)
end

function gen_connect(
    q::StateCar,  # to identify type
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle2D},
    eps::Float64;
    omega_max::Float64=OMEGA_MAX,
    step::Int64=10
    )::Function

    return (q_from::StateCar, q_to::StateCar, i::Int64) -> begin
        # check: valid moves
        (omega, t) = get_omega_t(q_from, q_to, omega_max)
        if !(abs(omega) <= omega_max && 0 <= t <= eps); return false; end

        for e=(0:step)/step
            # get intermediate state
            q = get_car_intermediate_state(q_from, omega, t, e)

            # check borders
            if !(rads[i] <= q.x <= 1 - rads[i] && rads[i] <= q.y <= 1 - rads[i])
                return false
            end

            # check: collisions with static obstacles
            if any(norm([o.x - q.x, o.y - q.y]) < rads[i] + o.r for o in obstacles)
                return false
            end
        end
        return true
    end
end

function gen_collide(
    q::StateCar,
    rads::Vector{Float64};
    omega_max::Float64=OMEGA_MAX,
    step::Int64=10)::Function

    N = length(rads)
    return (Q_from::Vector{Node{StateCar}}, Q_to::Vector{Node{StateCar}}) -> begin
        for i = 1:N, j = i+1:N
            q_from_i = Q_from[i].q
            q_from_j = Q_from[j].q
            (omega_i, t_i) = get_omega_t(q_from_i, Q_to[i].q, omega_max)
            (omega_j, t_j) = get_omega_t(q_from_j, Q_to[j].q, omega_max)
            for e_i=(0:step)/step, e_j=(0:step)/step
                q_i = get_car_intermediate_state(q_from_i, omega_i, t_i, e_i)
                q_j = get_car_intermediate_state(q_from_j, omega_j, t_j, e_j)
                if dist(q_i, q_j) < rads[i] + rads[j]
                    return true
                end
            end
        end
        return false
    end
end

function gen_random_walk(q::StateCar, eps::Float64; omega_max::Float64=OMEGA_MAX)::Function
    return (q::StateCar) -> begin
        omega = rand() * omega_max * 2 - omega_max
        t = rand() * eps
        x = q.x
        y = q.y
        theta = q.theta + t * omega
        if omega != 0
            x = q.x + (sin(omega * t + q.theta) - sin(q.theta)) / omega
            y = q.y + (-cos(omega * t + q.theta) + cos(q.theta)) / omega
        else
            x = q.x + t * cos(q.theta)
            y = q.y + t * sin(q.theta)
        end
        return StateCar(x, y, theta)
    end
end

function plot_motion!(
    q_from::StateCar, q_to::StateCar, rad::Float64, params; step::Int64=10)
    if q_from.x == q_to.x && q_from.y == q_to.y; return; end
    (omega, t) = get_omega_t(q_from, q_to)
    x_arr = Vector{Float64}()
    y_arr = Vector{Float64}()
    for e=(0:step)/step
        q = get_car_intermediate_state(q_from, omega, t, 1-e)
        pushfirst!(x_arr, q.x)
        pushfirst!(y_arr, q.y)
    end
    p = copy(params)
    if haskey(p, :markershape)
        scatter!([x_arr[end]], [y_arr[end]]; params...)
        delete!(p, :markershape)
    end
    plot!(x_arr, y_arr; p...)
end

function plot_start_goal!(q_init::StateCar, q_goal::StateCar, rad::Float64, params)
    plot!([q_init.x], [q_init.y]; markershape=:hex, params...)
    plot!([q_goal.x], [q_goal.y]; markershape=:star, params...)
end

function plot_agent!(q::StateCar, rad::Float64, color::String)
    plot_circle!(q.x, q.y, rad, color, 3.0, fillalpha=0.1)
    p_from_x = rad * cos(q.theta) + q.x
    p_from_y = rad * sin(q.theta) + q.y
    plot!([q.x, p_from_x], [q.y, p_from_y], lw=5, color=color, legend=nothing)
end

function expand!(
    V::Vector{Node{StateCar}},
    q::State,
    i::Int64,
    K::Int64,
    connect::Function,
    random_walk::Function
    ) where State <: AbsState
    for j = 1:K
        # sample one point
        q_new = random_walk(q)

        # check connection
        if !connect(q, q_new, i); continue; end

        # check the number of neighbors in candidates
        neighbors = filter(v -> dist(v.q, q_new) <= 0.2, V)
        if length(neighbors) >= K; continue; end

        # identify candidate neighbors
        C_v_u = filter(v -> connect(v.q, q_new, i), neighbors)
        C_u_v = filter(v -> connect(q_new, v.q, i), neighbors)

        # add vertex and edges
        u = Node(q_new, length(V) + 1, Vector{Int64}())
        push!(V, u)
        for v in C_v_u; push!(v.neighbors, u.id); end
        for v in C_u_v; push!(u.neighbors, v.id); end
    end
end

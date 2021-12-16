using MRMP
using JSON
using HTTP
import Random: seed!
import Printf: @printf, @sprintf
import Dates
import Sockets

PORT = UInt16(8081)
ADDR = "127.0.0.1"

struct Field
    x::Float64
    y::Float64
    size::Float64
end

scale(x::Real, y::Real, field::Field) = begin
    StatePoint2D((Float64(x) - field.x) / field.size, (Float64(y) - field.y) / field.size)
end
scale(x::Real, y::Real, r::Real, field::Field) = begin
    CircleObstacle2D((x - field.x) / field.size, (y - field.y) / field.size, r / field.size)
end
scale(r::Real, field::Field) = r / field.size
rescale(q::StatePoint2D, field::Field) = (q.x * field.size + field.x, q.y * field.size + field.y)
rescale_x(x::Float64, field::Field) = x * field.size + field.x
rescale_y(y::Float64, field::Field) = y * field.size + field.y

function get_instructions(TPG, field)
    get_dict(a) = Dict(
        "id" => a.id,
        "x_from" => rescale_x(a.from.q.x, field),
        "y_from" => rescale_y(a.from.q.y, field),
        "x_to" => rescale_x(a.to.q.x, field),
        "y_to" => rescale_y(a.to.q.y, field),
        "pre" => a.predecessors,
        "suc" => a.successors
    )
    return [[ get_dict(a) for a in actions ] for actions in TPG]
end

get_failure_msg() = JSON.json(Dict("status" => :failure))

function get_instance(req)
    field = Field(
        req["field"]["x"],
        req["field"]["y"],
        req["field"]["size"]
    )
    config_init = map(x -> scale(x["x_init"], x["y_init"], field), req["agents"])
    config_goal = map(x -> scale(x["x_goal"], x["y_goal"], field), req["agents"])
    rads = map(x -> scale(x["rad"], field), req["agents"])
    obstacles = (
        (req["obstacles"] != nothing)
        ? map(o -> scale(o["x"], o["y"], o["rad"], field), req["obstacles"])
        : Vector{CircleObstacle2D}()
    )
    return (field, config_init, config_goal, rads, obstacles)
end

macro fail_to_solve(msg::String)
    return esc(
        quote
            @warn @sprintf("%6.4f sec: %s\n", elapsed(), $msg)
            return get_failure_msg()
        end
    )
end

function solve(msg)
    req = JSON.parse(msg)

    # timeout utilities
    t_s = MRMP.now()
    time_limit = req["search"]["time_limit"]
    elapsed() = MRMP.elapsed_sec(t_s)
    timeover() = elapsed() > time_limit

    # setup instance
    (field, config_init, config_goal, rads, obstacles) = get_instance(req)
    print_instance(config_init, config_goal, rads, obstacles)
    if !is_valid_instance(config_init, config_goal, rads)
        @fail_to_solve("invalid instance")
    end

    # read search parameters
    seed!(haskey(req["search"], "seed") ? req["search"]["seed"] : 0)
    eps = req["search"]["eps"]
    num_neighbors = req["search"]["num_neighbors_init"]
    init_search_params = Dict([ (Symbol(key), val) for (key, val) in req["search"]["init"] ])
    refine_search_params = Dict([ (Symbol(key), val) for (key, val) in req["search"]["refine"] ])
    neighbors_growing_rate = (
        haskey(req["search"], "neighbors_growing_rate")
        ? req["search"]["neighbors_growing_rate"]
        : 1.1
    )

    # setup search details
    q = config_init[1]
    connect = gen_connect(q, rads, obstacles, eps)
    collide = gen_collide(q, rads)
    check_goal = gen_check_goal(config_goal)
    h_func = gen_h_func(config_goal)
    g_func = gen_g_func(greedy=true)
    random_walk = gen_random_walk(q, eps)
    get_sample_nums = gen_get_sample_nums(num_neighbors)

    get_roadmap_size() = sum([length(rmp) for rmp in roadmaps])

    # get initial solution
    @printf("searching initial solution\n")
    solution, roadmaps = search(
        config_init, config_goal, connect, collide, check_goal,
        h_func, g_func, random_walk, get_sample_nums;
        TIME_LIMIT=time_limit - elapsed(),
        init_search_params...)
    if solution == nothing; @fail_to_solve("failed to find initial solution"); end
    (TPG_best, solution_best, cost_best) = smoothing(solution, collide, connect)
    solution = solution_best
    if timeover(); @fail_to_solve("failed to find initial solution within time limit"); end
    @printf("- %8.4f sec: get initial solution, cost=%6.4f, roadmap=%d\n",
            elapsed(), cost_best, get_roadmap_size())

    # iterative refinement
    @printf("iterative refinement\n")
    iter_num = 0
    while !timeover()
        iter_num += 1
        if haskey(req["search"], "max_refine_num") && iter_num > req["search"]["max_refine_num"]
            break
        end

        num_neighbors = Int64(floor(num_neighbors * neighbors_growing_rate))
        get_sample_nums = gen_get_sample_nums(num_neighbors)
        tl = time_limit - elapsed()
        if haskey(req["search"], "each_refine_time_limit")
            tl = min(tl, req["search"]["each_refine_time_limit"])
        end
        solution_tmp = MRMP.refine!(
            config_init, config_goal, connect, collide, check_goal,
            solution_best, roadmaps, h_func, g_func, random_walk, get_sample_nums;
            TIME_LIMIT=tl,
            refine_search_params...)
        if solution_tmp == nothing;
            @printf("- %8.4f sec: iter=%4d,  refine timeout, roadmap=%6d\n",
                    elapsed(), iter_num, get_roadmap_size())
            continue
        end
        TPG, solution, cost = smoothing(solution_tmp, collide, connect)

        # update cost
        if cost < cost_best
            TPG_best, solution_best, cost_best = TPG, solution, cost
            @printf("- %8.4f sec: iter=%4d, update solution, cost=%6.4f, roadmap=%6d\n",
                    elapsed(), iter_num, cost_best, get_roadmap_size())
        else
            @printf("- %8.4f sec: iter=%4d,  no improvement, cost=%6.4f, roadmap=%6d\n",
                    elapsed(), iter_num, cost, get_roadmap_size())
        end
    end
    @printf("\n%8.4f sec: final cost=%6.4f\n", elapsed(), cost_best)

    if haskey(req["search"], "save_gif") && req["search"]["save_gif"]
        save = @task begin
            filename = "./local/toio2d/" * string(Dates.now()) * ".gif"
            plot_anim!(config_init, config_goal, obstacles, rads, solution_best; filename=filename, fps=3)
        end
        schedule(save)
    end
    MRMP.plot_tpg(TPG_best)
    MRMP.safe_savefig!("./local/tpg.pdf")

    return JSON.json(Dict(
        "status" => :success,
        "instructions" => get_instructions(TPG_best, field)
    ))
end

server = Sockets.listen(Sockets.getaddrinfo(ADDR), PORT)
Sockets.close() = close(server)
@async begin
    req_num = 0
    HTTP.WebSockets.listen(ADDR, PORT; server=server) do ws
        req_num += 1
        while !eof(ws)
            data = String(readavailable(ws))
            @printf("\n---\n%02d: receive request\n\n", req_num)
            instructions = solve(data)
            write(ws, instructions);
        end
    end
end

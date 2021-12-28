using MRMP
using JSON
using HTTP
import Random: seed!
import Printf: @printf, @sprintf
import Base: @kwdef
import Dates
import Sockets

PORT = UInt16(8081)
ADDR = "127.0.0.1"

struct Field
    x::Float64
    y::Float64
    size::Float64
end

@kwdef mutable struct Query
    field = nothing
    config_init = nothing
    config_goal = nothing
    rads = nothing
    obstacles = nothing
    roadmaps = nothing
    TPG = nothing
    TPG_committed = nothing
    solution = nothing
    cost = nothing
    connect = nothing
    collide = nothing
    check_goal = nothing
    h_func = nothing
    g_func = nothing
    random_walk = nothing
    num_neighbors = nothing
    neighbors_growing_rate = nothing
    init_search_params = nothing
    refine_search_params = nothing
    committed_indexes = nothing
    start_time = nothing
    save_fig = nothing
    hist_TPG = []
    hist_TPG_committed = []
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
get_commit_str(committed_indexes) = begin
    reduce(*, [@sprintf("%02d_", k) for k in committed_indexes])[1:end-1]
end

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

function save_planning(query)
    if !query.save_fig; return; end
    idx = length(query.hist_TPG)
    config_init = copy(query.config_init)
    solution = copy(query.solution)
    dirname = "./local/toio2d/"*string(query.start_time)
    datestr = string(Dates.now())
    @async begin
        TPG1 = query.hist_TPG_committed[idx]
        TPG2 = query.hist_TPG[idx]
        MRMP.plot_tpg!(TPG2; filename="$dirname/tpg/$datestr.pdf")
        MRMP.plot_tpg!(TPG1, TPG2; filename="$dirname/tpg_all/$datestr.pdf")
        plot_anim!(config_init, query.config_goal, query.obstacles, query.rads, solution;
                   filename="$dirname/anim/$datestr.gif", fps=3)
    end
end

function setup(req, query)
    # setup instance
    query.field = Field(req["field"]["x"], req["field"]["y"], req["field"]["size"])
    query.config_init = map(x -> scale(x["x_init"], x["y_init"], query.field), req["agents"])
    query.config_goal = map(x -> scale(x["x_goal"], x["y_goal"], query.field), req["agents"])
    query.rads = map(x -> scale(x["rad"], query.field), req["agents"])
    query.obstacles = (
        (req["obstacles"] != nothing)
        ? map(o -> scale(o["x"], o["y"], o["rad"], query.field), req["obstacles"])
        : Vector{CircleObstacle2D}()
    )

    # setup search details
    q = query.config_init[1]
    eps = req["search"]["eps"]
    query.connect = gen_connect(q, query.rads, query.obstacles, eps)
    query.collide = gen_collide(q, query.rads)
    query.check_goal = gen_check_goal(query.config_goal)
    query.h_func = gen_h_func(query.config_goal)
    query.g_func = gen_g_func(greedy=true)
    query.random_walk = gen_random_walk(q, eps)
    query.init_search_params = Dict([ (Symbol(key), val) for (key, val) in req["search"]["init"] ])
    query.refine_search_params = Dict([ (Symbol(key), val) for (key, val) in req["search"]["refine"] ])
    query.neighbors_growing_rate = (
        haskey(req["search"], "neighbors_growing_rate")
        ? req["search"]["neighbors_growing_rate"]
        : 1.1
    )
    query.num_neighbors = req["search"]["num_neighbors_init"]
    query.committed_indexes = fill(0, length(query.config_init))
    query.start_time = MRMP.now()
    query.save_fig = req["save_fig"]

    # setup seed
    seed!(haskey(req["search"], "seed") ? req["search"]["seed"] : 0)
end

macro fail_to_solve(msg::String)
    return esc(
        quote
            @warn @sprintf("%6.4f sec: %s\n", elapsed(), $msg)
            write(ws, JSON.json(Dict("status" => :failure)))
            return
        end
    )
end

function find_initial_instructions!(req, query, ws)
    N = length(query.config_init)

    # timeout utilities
    time_limit = req["search"]["time_limit"]
    elapsed() = MRMP.elapsed_sec(query.start_time)
    timeover() = elapsed() > time_limit

    print_instance(query.config_init, query.config_goal, query.rads, query.obstacles)
    if !is_valid_instance(query.config_init, query.config_goal, query.rads)
        @fail_to_solve("invalid instance")
    end

    # get initial solution
    @printf("searching initial solution\n")
    solution, query.roadmaps = search(
        query.config_init,
        query.config_goal,
        query.connect,
        query.collide,
        query.check_goal,
        query.h_func,
        query.g_func,
        query.random_walk,
        gen_get_sample_nums(query.num_neighbors);
        TIME_LIMIT=time_limit - elapsed(),
        query.init_search_params...)
    if solution == nothing; @fail_to_solve("initial solution is not found"); end

    # smoothing
    (query.TPG, query.solution, query.cost) = smoothing(solution, query.collide, query.connect;
                                                        skip_connection=false)
    @printf("- get initial solution, cost=%6.4f\n", query.cost)

    # send initial plan
    write(ws, JSON.json(Dict(
        "status" => :success,
        "instructions" => get_instructions(query.TPG, query.field)
    )))

    # save result
    query.TPG_committed = [Vector{MRMP.Action{StatePoint2D}}() for i=1:N]
    push!(query.hist_TPG, query.TPG)
    push!(query.hist_TPG_committed, query.TPG_committed)
    save_planning(query)

    # start refinement
    committed_indexes = [(length(actions[1].predecessors) == 0 ? 1 : 0) for actions in query.TPG]
    update_query_by_new_commit!(query, committed_indexes)
    refine_until_new_commit!(query, committed_indexes, ws)
end

function update_query_by_new_commit!(query, committed_indexes)
    @printf("commit %s: new commit\n", get_commit_str(committed_indexes))
    N = length(query.config_goal)

    # update TPG
    for i = 1:N
        k = committed_indexes[i] - query.committed_indexes[i]
        if k == 0 || length(query.TPG[i]) == 0; continue; end

        # save last commit
        l = length(query.TPG_committed[i])
        append!(query.TPG_committed[i], query.TPG[i][1:k])
        query.TPG_committed[i][l+1].t = sum(committed_indexes)
        if l > 0
            push!(query.TPG_committed[i][l].successors, (i, query.TPG_committed[i][l+1].id))
            push!(query.TPG_committed[i][l+1].predecessors, (i, query.TPG_committed[i][l].id))
        end

        # cut
        query.TPG[i] = query.TPG[i][k+1:end]
    end

    # remove unnecessary relationship
    for i in 1:N
        for action in query.TPG[i]
            del_list = []
            for (j, id) in action.predecessors
                if findfirst(a -> a.id == id, query.TPG[j]) == nothing
                    push!(del_list, (j, id))
                end
            end
            filter!(e -> !(e in del_list), action.predecessors)
        end
    end

    query.committed_indexes = committed_indexes
    query.config_init = [ (length(query.TPG[i]) > 0
                           ? query.TPG[i][1].from.q
                           : query.config_goal[i]) for i = 1:N ]
    query.solution = MRMP.get_greedy_solution(query.TPG; config_goal=query.config_goal)
    query.cost = MRMP.get_tpg_cost(query.TPG);
end

function refine_until_new_commit!(query, last_committed_indexes, ws)
    # utilities
    is_not_outdate() = (query.committed_indexes == last_committed_indexes)
    commit_name = get_commit_str(last_committed_indexes)

    @async begin
        # @printf("commit %s: start refinement\n", commit_name)
        iter_num = 0
        t_s = MRMP.now()
        while is_not_outdate() && MRMP.elapsed_sec(t_s) < 5
            sleep(0.001)
            iter_num += 1
            query.num_neighbors = Int64(floor(query.num_neighbors * query.neighbors_growing_rate))
            solution = MRMP.refine!(
                query.config_init,
                query.config_goal,
                query.connect,
                query.collide,
                query.check_goal,
                query.solution,
                query.roadmaps,
                query.h_func,
                query.g_func,
                query.random_walk,
                gen_get_sample_nums(query.num_neighbors);
                query.refine_search_params...)
            if solution == nothing; continue; end
            TPG, solution, cost = smoothing(solution, query.collide, query.connect;
                                            config_goal=query.config_goal)

            # update cost
            if cost < query.cost && is_not_outdate()
                @printf("commit %s: iter=%4d, update solution, cost=%6.4f -> %6.4f\n",
                        commit_name, iter_num, query.cost, cost)
                query.TPG, query.solution, query.cost = TPG, solution, cost
                write(ws, JSON.json(Dict(
                    "status" => :success,
                    "committed_indexes" => query.committed_indexes,
                    "instructions" => get_instructions(query.TPG, query.field)
                )))
                # store solutions
                push!(query.hist_TPG, query.TPG)
                push!(query.hist_TPG_committed, query.TPG_committed)
                save_planning(query)
            end
        end
        # @printf("commit %s: end refinement\n", commit_name)
    end
end

server = Sockets.listen(Sockets.getaddrinfo(ADDR), PORT)
Sockets.close() = close(server)
@async begin
    req_num = 0
    HTTP.WebSockets.listen(ADDR, PORT; server=server) do ws
        @printf("\n---\n%02d: receive request\n\n", req_num)
        req_num += 1
        query = Query()

        while !eof(ws)
            req = JSON.parse(String(readavailable(ws)))
            if req["type"] == "init"
                setup(req, query)
                find_initial_instructions!(req, query, ws)
            elseif req["type"] == "commit"
                # Node.js -> Julia
                committed_indexes = req["committed_indexes"] + fill(1, length(query.config_init))
                update_query_by_new_commit!(query, committed_indexes)
                refine_until_new_commit!(query, committed_indexes, ws)
            end
        end
        @printf("\n%02d: finish planning & execution\n", req_num)
    end
end

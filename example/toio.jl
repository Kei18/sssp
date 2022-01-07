using MRMP
using JSON
using HTTP
import Random: seed!
import Printf: @printf, @sprintf
import Base: @kwdef
import Sockets

include("./toio_util.jl")

function refine!(query::Query, config_init, solution, params)
    # update space filling metric
    query.num_neighbors = Int64(floor(query.num_neighbors * query.neighbors_growing_rate))
    # find new one using known solution
    solution_refined, roadmaps = search!(
        config_init,
        query.config_goal,
        query.connect,
        query.collide,
        query.check_goal,
        query.h_func,
        query.g_func,
        query.random_walk,
        gen_get_sample_nums(query.num_neighbors);
        current_solution = solution,
        roadmaps = query.roadmaps,
        params...,
    )
    return solution_refined
end

function find_initial_instructions!(req, query, ws)
    N = length(query.config_goal)
    print_instance(query.config_init, query.config_goal, query.rads, query.obstacles)
    if !is_valid_instance(query.config_init, query.config_goal, query.rads, query.obstacles)
        @fail_to_solve("invalid instance")
    end

    elapsed() = MRMP.elapsed_sec(query.start_time)
    timeover() = elapsed() > query.init_search_time_limit

    # get initial solution
    @info @sprintf("searching initial solution\n")
    solution, query.roadmaps = search!(
        query.config_init,
        query.config_goal,
        query.connect,
        query.collide,
        query.check_goal,
        query.h_func,
        query.g_func,
        query.random_walk,
        gen_get_sample_nums(query.num_neighbors);
        query.params_init...,
    )
    if solution == nothing
        @fail_to_solve("initial solution is not found")
    end

    # smoothing
    (TPG, solution, cost) = smoothing(solution, query.collide, query.connect)
    @info @sprintf("get initial solution, elapsed: %f sec, cost: %6.4f\n", elapsed(), cost)

    TPG_committed = [Vector{MRMP.Action{StatePoint2D}}() for i = 1:N]
    committed_indexes = fill(0, N)
    store_new_tpg(query, TPG, TPG_committed, committed_indexes)

    # check time constraints
    if timeover()
        @info @sprintf("reach timeout %f sec", query.init_search_time_limit)
    end

    # iterative refinement
    if !query.skip_init_refinement
        iter_refine = 0
        @info "start refinement:"
        while !timeover()
            iter_refine += 1
            # refinement
            solution_tmp = refine!(query, query.config_init, solution, query.params_refine_init)
            # fail to find solution -> retry
            if solution_tmp == nothing
                continue
            end
            # smoothing
            TPG_tmp, solution_tmp, cost_tmp =
                smoothing(solution_tmp, query.collide, query.connect)
            # replace
            if cost_tmp < cost
                @info @sprintf(
                    "\titer=%6d, elapsed: %f sec, cost=%6.4f -> %6.4f\n",
                    iter_refine,
                    elapsed(),
                    cost,
                    cost_tmp
                )
                TPG, solution, cost = TPG_tmp, solution_tmp, cost_tmp
                store_new_tpg(query, TPG, TPG_committed, committed_indexes)
            end
        end
    end

    # send initial plan
    write(
        ws,
        JSON.json(
            Dict(
                "status" => :success,
                "plan_id" => length(query.hist_TPG),
                "instructions" => get_instructions(TPG, query.field),
            ),
        ),
    )
end

function refine_until_new_commit!(req, query, ws; TIME_LIMIT::Real = 5)
    # setup TPG & TPG_committed
    plan_id = req["plan_id"]
    TPG = deepcopy(query.hist_TPG[plan_id])
    TPG_committed = deepcopy(query.hist_TPG_committed[plan_id])
    old_committed_indexes = deepcopy(query.hist_commited_indexes[plan_id])
    new_committed_indexes = map(k -> k + 1, req["committed_indexes"]) # Node.js -> Julia
    commit_name = get_commit_str(new_committed_indexes)
    setup_tpg!(TPG, TPG_committed, old_committed_indexes, new_committed_indexes)
    @info @sprintf("plan_id: %s, commit: %s, start refinement\n", plan_id, commit_name)

    # setup search details
    config_init =
        map(e -> (isempty(e[2]) ? query.config_goal[e[1]] : e[2][1].from.q), enumerate(TPG))

    solution = MRMP.get_greedy_solution(TPG; config_goal = query.config_goal)
    cost_original = MRMP.get_tpg_cost(TPG)

    # update committed indexes
    query.committed_indexes = new_committed_indexes

    @async begin
        t_s = MRMP.now()
        is_not_outdate() =
            query.committed_indexes == new_committed_indexes &&
            MRMP.elapsed_sec(t_s) < TIME_LIMIT

        while is_not_outdate()
            # to avoid busy loop
            sleep(0.001)
            # refinement
            solution = refine!(query, config_init, solution, query.params_refine_online)
            if solution == nothing
                continue
            end
            TPG, solution, cost = smoothing(solution, query.collide, query.connect)
            # better solution is found
            if cost < cost_original && is_not_outdate()

                @info @sprintf(
                    "commit %s: update solution, cost=%6.4f -> %6.4f\n",
                    commit_name,
                    cost_original,
                    cost
                )
                store_new_tpg(query, TPG, TPG_committed, new_committed_indexes)
                write(
                    ws,
                    JSON.json(
                        Dict(
                            "plan_id" => length(query.hist_TPG),
                            "committed_indexes" => new_committed_indexes,
                            "instructions" => get_instructions(TPG, query.field),
                        ),
                    ),
                )
                break
            end
        end
        @info @sprintf("commit %s: end refinement\n", commit_name)
    end
end

function main(; ADDR::String = "127.0.0.1", PORT = UInt16(8081))
    server = Sockets.listen(Sockets.getaddrinfo(ADDR), PORT)
    @async begin
        req_num = 0
        HTTP.WebSockets.listen(ADDR, PORT; server = server) do ws
            @info @sprintf("\n-----\n%02d: receive request", req_num)
            req_num += 1
            query = Query()

            while !eof(ws)
                try
                    req = JSON.parse(String(readavailable(ws)))
                    if req["type"] == "init"
                        setup(req, query)
                        find_initial_instructions!(req, query, ws)
                    elseif req["type"] == "commit" && !query.skip_online_refinement
                        refine_until_new_commit!(req, query, ws)
                    end
                catch e
                    println(e)
                end
            end
            @info @sprintf(
                "\n%02d: finish planning & execution, waiting for drawing tasks\n",
                req_num
            )
            for task in query.drawing_tasks
                wait(task)
            end
            @info @sprintf("all drawing tasks are completed")
        end
    end
    return server
end

server = main(; ARGS...)
Sockets.close() = close(server)

using MRMP
import Random: seed!
using Hyperopt
import YAML
import Printf: @printf, @sprintf
import Base.Threads
import Dates
import Logging


function main(config::Dict; pre_compile::Bool = false)

    # create result directory
    date_str = string(Dates.now())
    root_dir = joinpath(
        get(config, "root", joinpath(@__DIR__, "..", "..", "data", "exp")),
        date_str,
    )
    if !pre_compile
        @info @sprintf("result will be saved in %s", root_dir)
        !isdir(root_dir) && mkpath(root_dir)
        additional_info = Dict(
            "git_hash" => read(`git log -1 --pretty=format:"%H"`, String),
            "date" => date_str,
        )
        YAML.write_file(joinpath(root_dir, "config.yaml"), merge(config, additional_info))
    end

    # generate instance
    seed_offset = get(config, "seed_offset", 0)
    num_instances = get(config, "num_instances", 3)
    @info @sprintf("generating %d instances", num_instances)
    instances = begin
        params = Dict([(Symbol(key), val) for (key, val) in config["instance"]])
        delete!(params, Symbol("_target_"))
        target = Meta.parse(config["instance"]["_target_"])
        map(e -> begin
            seed!(e + seed_offset)
            eval(target)(; params...)
        end, 1:num_instances)
    end

    num_search_times = get(config, "num_search_times", 100)
    results = Dict()
    for solver_info in config["solvers"]
        solver_name = solver_info["_target_"]
        @info @sprintf(
            "hyper parameter search for %s with %d samples with %d threads",
            solver_name,
            num_search_times,
            Threads.nthreads()
        )
        params_key = collect(filter(key -> key != "_target_", keys(solver_info)))
        params_cands_str = Dict(map(key -> (Symbol(key), solver_info[key]), params_key))
        ho = Hyperoptimizer(num_search_times; params_cands_str...)
        for (i, params...) in ho
            score = Threads.Atomic{Float64}(0)
            Threads.@threads for k = 1:num_instances
                seed!(k)
                config_init, config_goal, obstacles, rads = instances[k]
                connect = gen_connect(config_init[1], rads, obstacles)
                collide = gen_collide(config_init[1], rads)
                check_goal = MRMP.gen_check_goal(config_goal)
                t = @elapsed begin
                    solution, roadmaps = eval(Meta.parse(solver_name))(
                        config_init,
                        config_goal,
                        connect,
                        collide,
                        check_goal;
                        TIME_LIMIT = config["time_limit"],
                        Dict(pairs(params))...,
                    )
                end
                isnothing(solution) && Threads.atomic_add!(score, 1.0 + t * 0.0001)
            end
            @info @sprintf(
                "fin: %04d/%04d, failure: %04d/%04d, params: %s\n",
                i,
                num_search_times,
                score[],
                num_instances,
                params
            )
            push!(ho.results, score[])
        end
        results[solver_name] = ho
    end

    if !pre_compile
        foreach(e -> @info(@sprintf("\nsolver:%s\n%s\n", e...)), results)
        YAML.write_file(
            joinpath(root_dir, "best_params.yaml"),
            Dict([
                (Symbol(solver_name), Dict(zip(ho.params, ho.minimizer))) for
                (solver_name, ho) in results
            ]),
        )
    end
end

function main(args::Vector{String})
    config_file = args[1]
    if !isfile(config_file)
        @warn @sprintf("%s does not exists", config_file)
        return
    end
    config = YAML.load_file(config_file)

    # run once to force compilation
    @info "pre-compilation"
    Logging.with_logger(Logging.SimpleLogger(stdout, Logging.Error)) do
        main(
            merge(config, Dict("num_instances" => 1, "num_search_times" => 1));
            pre_compile = true,
        )
    end

    # start experiment
    @info "done, start hypra search"
    main(config)
end

@time main(ARGS)

using MRMP
import Random: seed!
import CSV
import YAML
import Printf: @printf, @sprintf
import DataFrames: DataFrame
import Logging
import Dates
import Base.Threads
import Base.Threads: @threads


function main(config::Dict; pre_compile::Bool = false)

    # create result directory
    date_str = string(Dates.now())
    root_dir = joinpath(
        get(config, "root", joinpath(@__DIR__, "..", "..", "data", "exp")),
        date_str,
    )
    if !pre_compile
        @info @sprintf("result will be saved in %s", root_dir)
        if !isdir(root_dir)
            mkpath(root_dir)
        end
        additional_info = Dict(
            "git_hash" => read(`git log -1 --pretty=format:"%H"`, String),
            "date" => date_str,
        )
        YAML.write_file(joinpath(root_dir, "config.yaml"), merge(config, additional_info))
    end

    # instance generation
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
    if !pre_compile && get(config, "save_instance_images", false)
        @info "saving instance images"
        map(
            k -> plot_instance!(
                instances[k]...;
                filename = joinpath(root_dir, @sprintf("%04d_ins.png", k)),
            ),
            1:num_instances,
        )
    end

    num_solvers = length(config["solvers"])
    num_total_tasks = num_instances * num_solvers
    cnt_fin = Threads.Atomic{Int}(0)
    result = Array{Any}(undef, num_total_tasks)
    @info @sprintf("start solving with %d threads", Threads.nthreads())
    @threads for k = 1:num_instances
        config_init, config_goal, obstacles, rads = instances[k]

        # setup search details
        q = config_init[1]
        connect = gen_connect(q, rads, obstacles)
        collide = gen_collide(q, rads)
        check_goal = MRMP.gen_check_goal(config_goal)
        g_func = MRMP.gen_g_func(stay_penalty = 0.1)

        # solve
        for (l, solver_info) in enumerate(config["solvers"])
            target = Meta.parse(solver_info["_target_"])
            solver_name = string(target)
            params = Dict([(Symbol(key), val) for (key, val) in solver_info])
            delete!(params, Symbol("_target_"))
            seed!(k + seed_offset)
            t_s = MRMP.now()
            solution, roadmaps = eval(target)(
                config_init,
                config_goal,
                connect,
                collide,
                check_goal,
                g_func;
                params...,
            )
            result[num_solvers*(k-1)+l] = (
                instance = k,
                N = length(config_init),
                num_obs = length(obstacles),
                solver = solver_name,
                solver_index = l,
                elapsed_sec = MRMP.elapsed_sec(t_s),
                solved = !isnothing(solution),
            )
            Threads.atomic_add!(cnt_fin, 1)
        end

        @printf("\r%04d/%04d tasks have been finished", cnt_fin[], num_total_tasks)
    end
    println()

    # save result
    if !pre_compile
        @info("save result")
        CSV.write(joinpath(root_dir, "result.csv"), result)
    end
end

# read experimental setting
function main(args::Vector{String})
    config_file = args[1]
    if !isfile(config_file)
        @warn @sprintf("%s does not exists", config_file)
        return
    end
    config = YAML.load_file(config_file)

    # parse arguments
    for k = 2:length(args)
        keys, val = split(args[k], "=")
        _config = config
        keys_arr = split(keys, ".")
        for key in keys_arr[1:end-1]
            if haskey(_config, key)
                _config = _config[key]
            else
                @error @sprintf(
                    "%s does not have key %s",
                    config_file,
                    join(keys_arr[1:end-1], ".")
                )
                return
            end
        end

        # parse type
        val_parsed = tryparse(Int64, val)
        if isnothing(val_parsed)
            val_parsed = tryparse(Float64, val)
        end
        if isnothing(val_parsed)
            val_parsed = val
        end

        _config[keys_arr[end]] = val_parsed
    end

    # run once to force compilation
    @info "pre-compilation"
    Logging.with_logger(Logging.SimpleLogger(stdout, Logging.Error)) do
        main(merge(config, Dict("num_instances" => 1)); pre_compile = true)
    end

    # start experiment
    @info "done, start performance measurement"
    main(config)
end

@time main(ARGS)
